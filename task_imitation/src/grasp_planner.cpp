#include "task_imitation/grasp_planner.h"

#include <limits.h>
#include <math.h>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/random_sample.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rapid_collision/collision_checks.h"
#include "rapid_ros/params.h"
#include "rapid_viz/axes_markers.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "task_perception/pcl_typedefs.h"
#include "task_perception/pcl_utils.h"
#include "task_perception/pr2_gripper_model.h"
#include "task_perception_msgs/DemoState.h"
#include "transform_graph/graph.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_imitation/grasp_planning_context.h"

namespace tg = transform_graph;
namespace msgs = task_perception_msgs;
using geometry_msgs::Pose;
using rapid::GetDoubleParamOrThrow;

namespace pbi {
GraspFeatures::GraspFeatures()
    : antipodal_grasp_pts(0),
      non_antipodal_grasp_pts(0),
      antipodal_collisions(0),
      non_antipodal_collisions(0),
      sq_wrist_distance(0),
      is_colliding_with_obstacle(false) {}

GraspFeatureWeights::GraspFeatureWeights()
    : antipodal_grasp_weight(0),
      non_antipodal_grasp_weight(0),
      antipodal_collision_weight(0),
      non_antipodal_collision_weight(0),
      sq_wrist_distance_weight(0),
      obstacle_collision_weight(0) {}

GraspEvaluation::GraspEvaluation() : features(), weights() {}

std::string GraspEvaluation::ToString() const {
  std::stringstream ss;
  ss << "score: " << score() << " = " << weights.antipodal_grasp_weight << "*"
     << features.antipodal_grasp_pts << " + "
     << weights.non_antipodal_grasp_weight << "*"
     << features.non_antipodal_grasp_pts << " + "
     << weights.antipodal_collision_weight << "*"
     << features.antipodal_collisions << " + "
     << weights.non_antipodal_collision_weight << "*"
     << features.non_antipodal_collisions << " + "
     << weights.sq_wrist_distance_weight << "*" << features.sq_wrist_distance
     << " = " << weights.antipodal_grasp_weight * features.antipodal_grasp_pts
     << " + "
     << weights.non_antipodal_grasp_weight * features.non_antipodal_grasp_pts
     << " + "
     << weights.antipodal_collision_weight * features.antipodal_collisions
     << " + "
     << weights.non_antipodal_collision_weight *
            features.non_antipodal_collisions
     << " + " << weights.sq_wrist_distance_weight * features.sq_wrist_distance;
  if (features.is_colliding_with_obstacle) {
    ss << " (collision)";
  } else {
    ss << " (safe)";
  }
  return ss.str();
}

double GraspEvaluation::score() const {
  double score = 0;
  score += weights.antipodal_grasp_weight * features.antipodal_grasp_pts;
  score +=
      weights.non_antipodal_grasp_weight * features.non_antipodal_grasp_pts;
  score += weights.antipodal_collision_weight * features.antipodal_collisions;
  score += weights.non_antipodal_collision_weight *
           features.non_antipodal_collisions;
  score += weights.sq_wrist_distance_weight * features.sq_wrist_distance;
  if (features.is_colliding_with_obstacle) {
    score += weights.obstacle_collision_weight;
  }
  return score;
}

ScoredGrasp::ScoredGrasp()
    : score(-std::numeric_limits<double>::max()), pose() {}

bool ScoredGrasp::IsValid() {
  return score != -std::numeric_limits<double>::max();
}

GraspPlanner::GraspPlanner()
    : nh_(),
      gripper_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "grasp_planner/grippers", 20, true)),
      object_pub_(nh_.advertise<sensor_msgs::PointCloud2>(
          "grasp_planner/object", 1, true)),
      kGripperMarkers(),
      debug_(false),
      debug_cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>(
          "grasp_planner/cloud", 1, true)) {
  InitGripperMarkers();
}

Pose GraspPlanner::Plan(const GraspPlanningContext& context) {
  UpdateParams();
  Pose wrist_pose = context.wrist_pose();
  if (debug_) {
    visualization_msgs::MarkerArray wrist_axes = rapid::AxesMarkerArray(
        "wrist", context.planning_frame_id(), wrist_pose, 0.1);
    gripper_pub_.publish(wrist_axes);
  }

  Pr2GripperModel gripper_model;
  gripper_model.set_pose(wrist_pose);
  VisualizeGripper("optimization", wrist_pose, context.planning_frame_id());

  PublishPointCloud(object_pub_, *context.object_cloud());

  Pose initial_pose = ComputeInitialGrasp(gripper_model, context);
  gripper_model.set_pose(initial_pose);
  VisualizeGripper("optimization", initial_pose, context.planning_frame_id());

  Pose to_wrist_pose = OrientTowardsWrist(gripper_model, context);
  gripper_model.set_pose(to_wrist_pose);
  VisualizeGripper("optimization", to_wrist_pose, context.planning_frame_id());
  if (debug_) {
    // ROS_INFO("Initial grasp");
    // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
  }

  return Plan(to_wrist_pose, context);
}

Pose GraspPlanner::Plan(const Pose& initial_pose,
                        const GraspPlanningContext& context) {
  UpdateParams();

  const int kMaxPlacementIterations =
      rapid::GetIntParamOrThrow("grasp_planner/max_placement_iterations");

  Pr2GripperModel gripper_model;
  gripper_model.set_pose(initial_pose);

  PublishPointCloud(object_pub_, *context.object_cloud());
  if (debug_) {
    visualization_msgs::MarkerArray wrist_axes = rapid::AxesMarkerArray(
        "wrist", context.planning_frame_id(), context.wrist_pose(), 0.1);
    while (ros::ok() && gripper_pub_.getNumSubscribers() == 0) {
      ROS_INFO_THROTTLE(1,
                        "Waiting for Rviz to subscribe to gripper markers...");
      ros::Duration(0.1).sleep();
    }
    gripper_pub_.publish(wrist_axes);
  }

  // Sample some points
  int num_samples = context.object_cloud()->size() * 0.1;
  boost::shared_ptr<std::vector<int> > sample_indices(new std::vector<int>());
  *sample_indices = SampleObject(context, num_samples);
  if (debug_) {
    PublishPointCloud(debug_cloud_pub_, context.object_cloud(), sample_indices);
  }
  ROS_INFO("Sampled %d points", num_samples);

  ScoredGrasp best;
  for (int i = num_samples - 1; i >= 0; --i) {
    const int sample_index = sample_indices->at(i);

    // Visualize the object normal
    if (debug_) {
      VisualizePointNormal(context, sample_index);
    }

    Pr2GripperModel model;
    model.set_pose(gripper_model.pose());

    Pose centered = CenterGraspOnPoint(model, context, sample_index);
    model.set_pose(centered);

    Pose aligned = AlignGraspWithPoint(model, context, sample_index);
    if (debug_) {
      VisualizeGripper("optimization", aligned, context.planning_frame_id());
      // ROS_INFO("Aligned with normal");
      // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
    }

    Pose placed = OptimizePlacement(aligned, context, kMaxPlacementIterations);
    if (debug_) {
      VisualizeGripper("optimization", placed, context.planning_frame_id());
      // ROS_INFO("Placed");
      // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
    }

    model.set_pose(placed);
    ScoredGrasp grasp = OptimizePitch(model, context);
    if (debug_) {
      VisualizeGripper("optimization", grasp.pose, context.planning_frame_id());
      // ROS_INFO("Pitched");
      // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
    }

    model.set_pose(grasp.pose);
    if (IsGripperCollidingWithObstacles(model, context)) {
      ScoredGrasp escaped = EscapeCollision(model, context);
      if (escaped.IsValid()) {
        if (debug_) {
          ROS_INFO("Escaped");
        }
        grasp = escaped;
      } else {
        if (debug_) {
          ROS_WARN("Unable to escape collision");
        }
      }
      if (debug_) {
        VisualizeGripper("optimization", grasp.pose,
                         context.planning_frame_id());
      }
    }

    if (grasp.score > best.score) {
      best = grasp;
      ROS_INFO("Best score: %f", best.score);
    }
    if (debug_) {
      VisualizeGripper("optimization_best", best.pose,
                       context.planning_frame_id());
      // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
    }
  }
  VisualizeGripper("optimization_best", best.pose, context.planning_frame_id());
  if (best.IsValid()) {
    ROS_INFO("Planned grasp with score: %f", best.score);
    return best.pose;
  } else {
    ROS_ERROR("Unable to plan grasp");
    Pose blank;
    return blank;
  }
}

void GraspPlanner::InitGripperMarkers() {
  urdf::Model model;
  model.initParam("robot_description");
  robot_markers::Builder builder(model);
  builder.Init();
  builder.SetNamespace("gripper");

  std::map<std::string, double> joint_positions;
  joint_positions["l_gripper_joint"] = 0.088;
  joint_positions["l_gripper_l_finger_joint"] = 0.514;
  joint_positions["l_gripper_l_finger_tip_joint"] = 0.514;
  joint_positions["l_gripper_r_finger_joint"] = 0.514;
  joint_positions["l_gripper_r_finger_tip_joint"] = 0.514;
  builder.SetJointPositions(joint_positions);

  std::set<std::string> gripper_links;
  gripper_links.insert("l_gripper_palm_link");
  gripper_links.insert("l_gripper_l_finger_link");
  gripper_links.insert("l_gripper_l_finger_tip_link");
  gripper_links.insert("l_gripper_r_finger_link");
  gripper_links.insert("l_gripper_r_finger_tip_link");

  builder.Build(gripper_links, &kGripperMarkers);

  // Shift palm to origin / identity orientation.
  Pose root_pose;
  for (size_t i = 0; i < kGripperMarkers.markers.size(); ++i) {
    if (kGripperMarkers.markers[i].mesh_resource.find("palm") !=
        std::string::npos) {
      root_pose = kGripperMarkers.markers[i].pose;
      break;
    }
  }
  Eigen::Affine3d gripper_pose;
  tf::poseMsgToEigen(root_pose, gripper_pose);

  for (size_t i = 0; i < kGripperMarkers.markers.size(); ++i) {
    visualization_msgs::Marker& marker = kGripperMarkers.markers[i];
    Eigen::Affine3d marker_pose;
    tf::poseMsgToEigen(marker.pose, marker_pose);
    Eigen::Affine3d shifted_pose = gripper_pose.inverse() * marker_pose;
    tf::poseEigenToMsg(shifted_pose, marker.pose);
  }
}

void GraspPlanner::VisualizeGripper(const std::string& ns, const Pose& pose,
                                    const std::string& frame_id) {
  visualization_msgs::MarkerArray marker_arr;
  Pr2GripperModel model;
  model.set_pose(pose);
  model.ToMarkerArray(frame_id, &marker_arr);
  for (size_t i = 0; i < marker_arr.markers.size(); ++i) {
    marker_arr.markers[i].ns = ns + "_model";
  }

  Eigen::Affine3d pose_transform;
  tf::poseMsgToEigen(pose, pose_transform);
  for (size_t i = 0; i < kGripperMarkers.markers.size(); ++i) {
    visualization_msgs::Marker marker = kGripperMarkers.markers[i];
    marker.header.frame_id = frame_id;
    marker.ns = ns;

    Eigen::Affine3d marker_pose;
    tf::poseMsgToEigen(marker.pose, marker_pose);
    Eigen::Affine3d shifted_pose = pose_transform * marker_pose;
    tf::poseEigenToMsg(shifted_pose, marker.pose);
    marker_arr.markers.push_back(marker);
  }

  gripper_pub_.publish(marker_arr);
}

Pose GraspPlanner::ComputeInitialGrasp(const Pr2GripperModel& gripper_model,
                                       const GraspPlanningContext& context) {
  Eigen::Vector3d grasp_center = gripper_model.grasp_center();

  // Initial pose to optimize around: translate gripper such that the closest
  // object point is in the center.
  KdTreeP::Ptr object_tree = context.object_tree();
  PointP grasp_center_pt;
  grasp_center_pt.x = grasp_center.x();
  grasp_center_pt.y = grasp_center.y();
  grasp_center_pt.z = grasp_center.z();
  std::vector<int> indices;
  std::vector<float> sq_distances;
  object_tree->nearestKSearch(grasp_center_pt, 1, indices, sq_distances);
  PointP nearest_obj_pt = object_tree->getInputCloud()->at(indices[0]);

  Eigen::Vector3d nearest_obj_vec;
  nearest_obj_vec << nearest_obj_pt.x, nearest_obj_pt.y, nearest_obj_pt.z;
  Eigen::Vector3d translation = nearest_obj_vec - gripper_model.grasp_center();

  Pose initial_pose = gripper_model.pose();
  initial_pose.position.x += translation.x();
  initial_pose.position.y += translation.y();
  initial_pose.position.z += translation.z();
  return initial_pose;
}

std::vector<int> GraspPlanner::SampleObject(const GraspPlanningContext& context,
                                            int num_samples) {
  pcl::RandomSample<pcl::PointXYZ> random;
  random.setInputCloud(context.object_cloud());
  random.setSample(num_samples);
  random.setSeed(0);
  std::vector<int> indices;
  random.filter(indices);
  return indices;
}

void GraspPlanner::VisualizePointNormal(const GraspPlanningContext& context,
                                        const int index) {
  const pcl::PointNormal& obj_pt =
      context.object_cloud_with_normals()->at(index);
  Eigen::Vector3d normal(obj_pt.normal_x, obj_pt.normal_y, obj_pt.normal_z);
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = context.planning_frame_id();
  arrow.ns = "normals";
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.scale.x = 0.0025;
  arrow.scale.y = 0.005;
  arrow.color.g = 1;
  arrow.color.a = 1;
  geometry_msgs::Point normal_start;
  normal_start.x = obj_pt.x;
  normal_start.y = obj_pt.y;
  normal_start.z = obj_pt.z;
  geometry_msgs::Point normal_end;
  normal_end.x = obj_pt.x + obj_pt.normal_x * 0.025;
  normal_end.y = obj_pt.y + obj_pt.normal_y * 0.025;
  normal_end.z = obj_pt.z + obj_pt.normal_z * 0.025;
  arrow.points.push_back(normal_start);
  arrow.points.push_back(normal_end);
  visualization_msgs::MarkerArray normal_arr;
  normal_arr.markers.push_back(arrow);
  gripper_pub_.publish(normal_arr);
}

Pose GraspPlanner::CenterGraspOnPoint(const Pr2GripperModel& gripper_model,
                                      const GraspPlanningContext& context,
                                      const int index) {
  Eigen::Affine3d pose_affine;
  tf::poseMsgToEigen(gripper_model.pose(), pose_affine);

  Eigen::Vector3d grasp_center = gripper_model.grasp_center();

  const pcl::PointXYZ& obj_pt = context.object_cloud()->at(index);
  Eigen::Vector3d obj_pt_vec(obj_pt.x, obj_pt.y, obj_pt.z);
  pose_affine.pretranslate(obj_pt_vec - grasp_center);

  Pose translated;
  tf::poseEigenToMsg(pose_affine, translated);
  return translated;
}

Pose GraspPlanner::AlignGraspWithPoint(const Pr2GripperModel& gripper_model,
                                       const GraspPlanningContext& context,
                                       const int index) {
  // Compute rotation
  const pcl::PointNormal& obj_pt =
      context.object_cloud_with_normals()->at(index);
  Eigen::Vector3d normal(obj_pt.normal_x, obj_pt.normal_y, obj_pt.normal_z);
  Eigen::Quaterniond grasp_orientation;
  tf::quaternionMsgToEigen(gripper_model.pose().orientation, grasp_orientation);

  Eigen::Vector3d gripper_y_axis = grasp_orientation.toRotationMatrix().col(1);
  Eigen::AngleAxisd rotation1(
      Eigen::Quaterniond::FromTwoVectors(gripper_y_axis, normal));
  Eigen::AngleAxisd rotation2(
      Eigen::Quaterniond::FromTwoVectors(gripper_y_axis, -normal));
  bool is_rotation1 = rotation1.angle() < rotation2.angle();
  Eigen::Quaterniond rotation(is_rotation1 ? rotation1 : rotation2);

  // Compute pose of the grasp center after rotation
  tg::Graph graph = gripper_model.tf_graph();
  tg::Transform center_in_planning;
  graph.ComputeDescription("grasp center", tg::RefFrame("gripper base"),
                           &center_in_planning);
  Eigen::Affine3d center_affine = center_in_planning.affine();
  center_affine.pretranslate(-gripper_model.grasp_center());
  center_affine.prerotate(rotation);
  center_affine.pretranslate(gripper_model.grasp_center());
  graph.Add("rotated grasp center", tg::RefFrame("gripper base"),
            center_affine);

  // Get gripper pose to match the rotated grasp center
  tg::Transform gripper_in_grasp_center;
  graph.ComputeDescription("gripper", tg::RefFrame("grasp center"),
                           &gripper_in_grasp_center);
  tg::Transform rotated_tf;
  graph.DescribePose(gripper_in_grasp_center,
                     tg::Source("rotated grasp center"),
                     tg::Target("gripper base"), &rotated_tf);
  return rotated_tf.pose();
}

ScoredGrasp GraspPlanner::OptimizePitch(const Pr2GripperModel& gripper_model,
                                        const GraspPlanningContext& context) {
  // Compute number of samples
  const double kDegToRad = M_PI / 180;
  const double pitch_range =
      kDegToRad * GetDoubleParamOrThrow("grasp_planner/pitch_range_degrees");
  const double half_pitch_range = pitch_range / 2;
  const double pitch_resolution =
      GetDoubleParamOrThrow("grasp_planner/pitch_resolution_degrees") *
      kDegToRad;
  const int num_pitch_samples = round(pitch_range / pitch_resolution) + 1;

  // Get wrist pose
  const geometry_msgs::Pose& wrist_pose = context.wrist_pose();
  Eigen::Vector3d wrist_pos;
  wrist_pos << wrist_pose.position.x, wrist_pose.position.y,
      wrist_pose.position.z;

  // Get transform graph and starter transforms
  tg::Graph tf_graph = gripper_model.tf_graph();
  tg::Transform gripper_in_grasp_center;
  tf_graph.ComputeDescription("gripper", tg::RefFrame("grasp center"),
                              &gripper_in_grasp_center);

  ScoredGrasp best;
  for (int pitch_i = 0; pitch_i < num_pitch_samples; ++pitch_i) {
    double pitch_angle = pitch_i * pitch_resolution - half_pitch_range;
    Eigen::AngleAxisd pitch_rot(pitch_angle, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond rotation(pitch_rot);

    tf_graph.Add("rotated grasp center", tg::RefFrame("grasp center"),
                 tg::Transform(tg::Position(), rotation));
    // New gripper pose in camera frame, after rotation.
    tg::Transform rotated_tf;
    tf_graph.DescribePose(gripper_in_grasp_center,
                          tg::Source("rotated grasp center"),
                          tg::Target("gripper base"), &rotated_tf);
    Pose rotated_pose = rotated_tf.pose();
    Eigen::Affine3d rotated_mat(rotated_tf.matrix());

    Pr2GripperModel candidate;
    candidate.set_pose(rotated_pose);
    if (debug_) {
      VisualizeGripper("optimization", rotated_pose,
                       context.planning_frame_id());
      ros::Duration(0.01).sleep();
    }

    // Optimize soft constraints
    GraspEvaluation grasp_eval = ScoreGrasp(candidate, wrist_pos, context);
    double score = grasp_eval.score();

    if (score > best.score) {
      best.score = score;
      best.pose = rotated_pose;
      if (debug_) {
        ROS_INFO("p: %f: %s", pitch_angle * 180 / M_PI,
                 grasp_eval.ToString().c_str());
      }
    }
  }
  return best;
}

ScoredGrasp GraspPlanner::EscapeCollision(const Pr2GripperModel& gripper_model,
                                          const GraspPlanningContext& context) {
  Eigen::Affine3d pose_affine;
  tf::poseMsgToEigen(gripper_model.pose(), pose_affine);

  // Get wrist pose
  const geometry_msgs::Pose& wrist_pose = context.wrist_pose();
  Eigen::Vector3d wrist_pos;
  wrist_pos << wrist_pose.position.x, wrist_pose.position.y,
      wrist_pose.position.z;

  const int max_escape_attempts =
      rapid::GetIntParamOrThrow("grasp_planner/max_escape_attempts");
  ScoredGrasp best;
  best.pose = gripper_model.pose();
  for (int i = 0; i < max_escape_attempts; ++i) {
    // Randomly translate gripper
    Eigen::Vector3d rand(Eigen::Vector3d::Random(3));
    rand[0] *= Pr2GripperModel::kGraspRegionDims.x / 2;
    rand[1] *= Pr2GripperModel::kGraspRegionDims.y / 2;
    rand[2] *= Pr2GripperModel::kGraspRegionDims.z / 2;
    Eigen::Affine3d moved = pose_affine;
    moved.translate(rand);

    Pose moved_pose;
    tf::poseEigenToMsg(moved, moved_pose);
    if (debug_) {
      VisualizeGripper("optimization", moved_pose, context.planning_frame_id());
    }

    // Check collisions with obstacles
    Pr2GripperModel model;
    model.set_pose(moved_pose);
    if (IsGripperCollidingWithObstacles(model, context)) {
      continue;
    }

    // Check collisions with object
    PointCloudP::Ptr object_cloud = context.object_cloud();
    PointCloudP::Ptr transformed_cloud(new PointCloudP);
    pcl::transformPointCloud(*object_cloud, *transformed_cloud,
                             moved.inverse());
    if (NumCollisions(transformed_cloud) > 0) {
      continue;
    }

    // If we are not colliding with anything, return.
    GraspEvaluation grasp_eval = ScoreGrasp(model, wrist_pos, context);
    double score = grasp_eval.score();

    if (score > best.score) {
      best.score = score;
      best.pose = moved_pose;
      if (debug_) {
        ROS_INFO("[Escape] Best score so far: %s",
                 grasp_eval.ToString().c_str());
      }
    }
  }

  return best;
}

Pose GraspPlanner::OrientTowardsWrist(const Pr2GripperModel& gripper_model,
                                      const GraspPlanningContext& context) {
  // Rotate the gripper about the center of the grasp region (CoG) such that the
  // centerline of the gripper is coincident with the line between the wrist and
  // the CoG.
  tg::Graph tf_graph = gripper_model.tf_graph();

  // Vector from CoG to wrist
  const Pose& wrist_pose = context.wrist_pose();
  Eigen::Vector3d wrist_vec;
  wrist_vec << wrist_pose.position.x, wrist_pose.position.y,
      wrist_pose.position.z;
  tg::Position wrist_in_grasp_center;
  tf_graph.DescribePosition(wrist_vec, tg::Source("gripper base"),
                            tg::Target("grasp center"), &wrist_in_grasp_center);
  Eigen::Vector3d desired_line = wrist_in_grasp_center.vector();

  // Centerline
  tg::Transform gripper_in_grasp_center;
  tf_graph.ComputeDescription(tg::LocalFrame("gripper"),
                              tg::RefFrame("grasp center"),
                              &gripper_in_grasp_center);
  Eigen::Vector3d centerline =
      gripper_in_grasp_center.matrix().topRightCorner(3, 1);

  // Rotation (about the grasp center)
  // Add a rotated frame to where the grasp center is
  // Then, map the transform from the CoG to the center
  Eigen::Quaterniond rotation =
      Eigen::Quaterniond::FromTwoVectors(centerline, desired_line);
  tf_graph.Add("rotated grasp center", tg::RefFrame("grasp center"),
               tg::Transform(tg::Position(), rotation));

  tg::Transform out;
  tf_graph.DescribePose(gripper_in_grasp_center,
                        tg::Source("rotated grasp center"),
                        tg::Target("gripper base"), &out);
  return out.pose();
}

Pose GraspPlanner::OptimizeOrientation(const Pr2GripperModel& gripper_model,
                                       const GraspPlanningContext& context) {
  // Sample roll and yaw angles (about the center of the grasp)
  // Get transform describing the gripper in the grasp center and cache it.
  tg::Graph tf_graph = gripper_model.tf_graph();
  tg::Transform gripper_in_grasp_center;
  tf_graph.ComputeDescription(tg::LocalFrame("gripper"),
                              tg::RefFrame("grasp center"),
                              &gripper_in_grasp_center);

  tg::Transform grasp_center_in_camera;
  tf_graph.ComputeDescription(tg::LocalFrame("grasp center"),
                              tg::RefFrame("gripper base"),
                              &grasp_center_in_camera);

  const geometry_msgs::Pose& wrist_pose = context.wrist_pose();
  Eigen::Vector3d wrist_pos;
  wrist_pos << wrist_pose.position.x, wrist_pose.position.y,
      wrist_pose.position.z;

  Pose best_pose = gripper_model.pose();
  GraspEvaluation initial_eval = ScoreGrasp(gripper_model, wrist_pos, context);
  double best_score = initial_eval.score();
  if (debug_) {
    ROS_INFO("Initial score: %s", initial_eval.ToString().c_str());
  }

  const double kDegToRad = M_PI / 180;
  const double kYawRange =
      kDegToRad * GetDoubleParamOrThrow("grasp_planner/yaw_range_degrees");
  const double kYawResolution =
      kDegToRad * GetDoubleParamOrThrow("grasp_planner/yaw_resolution_degrees");
  const double kRollRange =
      kDegToRad * GetDoubleParamOrThrow("grasp_planner/roll_range_degrees");
  const double kRollResolution =
      kDegToRad *
      GetDoubleParamOrThrow("grasp_planner/roll_resolution_degrees");
  const double kPitchRange =
      kDegToRad * GetDoubleParamOrThrow("grasp_planner/pitch_range_degrees");
  const double kPitchResolution =
      GetDoubleParamOrThrow("grasp_planner/pitch_resolution_degrees") *
      kDegToRad;
  int num_yaw_samples = round(kYawRange / kYawResolution) + 1;
  int num_roll_samples = round(kRollRange / kRollResolution) + 1;
  int num_pitch_samples = round(kPitchRange / kPitchResolution) + 1;
  for (int yaw_i = 0; yaw_i < num_yaw_samples; ++yaw_i) {
    double yaw_angle = yaw_i * kYawResolution - kYawRange / 2;
    Eigen::AngleAxisd yaw_rot(yaw_angle, Eigen::Vector3d::UnitZ());
    for (int roll_i = 0; roll_i < num_roll_samples; ++roll_i) {
      double roll_angle = roll_i * kRollResolution - kRollRange / 2;
      // Compute rotation about the grasp center.
      Eigen::AngleAxisd roll_rot(roll_angle, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond rotation = yaw_rot * roll_rot;

      tf_graph.Add("rotated grasp center", tg::RefFrame("grasp center"),
                   tg::Transform(tg::Position(), rotation));
      // New gripper pose in camera frame, after rotation.
      tg::Transform rotated_tf;
      tf_graph.DescribePose(gripper_in_grasp_center,
                            tg::Source("rotated grasp center"),
                            tg::Target("gripper base"), &rotated_tf);
      Pose rotated_pose;
      rotated_tf.ToPose(&rotated_pose);
      Eigen::Affine3d rotated_mat(rotated_tf.matrix());

      Pr2GripperModel candidate;
      candidate.set_pose(rotated_pose);

      GraspEvaluation grasp_eval = ScoreGrasp(candidate, wrist_pos, context);
      double score = grasp_eval.score();

      if (score > best_score) {
        best_score = score;
        best_pose = rotated_pose;
        if (debug_) {
          VisualizeGripper("optimization", rotated_pose,
                           context.planning_frame_id());
          ROS_INFO("y: %f r: %f: %s", yaw_angle * 180 / M_PI,
                   roll_angle * 180 / M_PI, grasp_eval.ToString().c_str());
        }
      }
    }
  }

  tf_graph.Add("gripper", tg::RefFrame("gripper base"), best_pose);

  for (int pitch_i = 0; pitch_i < num_pitch_samples; ++pitch_i) {
    double pitch_angle = pitch_i * kPitchResolution - kPitchRange / 2;
    Eigen::AngleAxisd pitch_rot(pitch_angle, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond rotation(pitch_rot);

    tf_graph.Add("rotated grasp center", tg::RefFrame("grasp center"),
                 tg::Transform(tg::Position(), rotation));
    // New gripper pose in camera frame, after rotation.
    tg::Transform rotated_tf;
    tf_graph.DescribePose(gripper_in_grasp_center,
                          tg::Source("rotated grasp center"),
                          tg::Target("gripper base"), &rotated_tf);
    Pose rotated_pose;
    rotated_tf.ToPose(&rotated_pose);
    Eigen::Affine3d rotated_mat(rotated_tf.matrix());

    Pr2GripperModel candidate;
    candidate.set_pose(rotated_pose);

    GraspEvaluation grasp_eval = ScoreGrasp(candidate, wrist_pos, context);
    double score = grasp_eval.score();

    if (score > best_score) {
      best_score = score;
      best_pose = rotated_pose;
      if (debug_) {
        VisualizeGripper("optimization", rotated_pose,
                         context.planning_frame_id());
        ROS_INFO("p: %f: %s", pitch_angle * 180 / M_PI,
                 grasp_eval.ToString().c_str());
      }
    }
  }

  return best_pose;
}

GraspEvaluation GraspPlanner::ScoreGrasp(const Pr2GripperModel& model,
                                         const Eigen::Vector3d& wrist_pos,
                                         const GraspPlanningContext& context) {
  GraspEvaluation eval;
  eval.weights = weights_;

  // Transform object points into rotated gripper frame.
  Eigen::Affine3d pose_affine;
  tf::poseMsgToEigen(model.pose(), pose_affine);
  PointCloudN::Ptr object_cloud = context.object_cloud_with_normals();
  PointCloudN::Ptr transformed_cloud(new PointCloudN);
  pcl::transformPointCloudWithNormals(*object_cloud, *transformed_cloud,
                                      pose_affine.inverse());

  // For all points in the grasp region, measure antipodality (i.e., how
  // collinear the normal is with the finger normals). Instead of
  // averaging, we count the number of normals that are antipodal "enough."
  for (size_t i = 0; i < transformed_cloud->size(); ++i) {
    PointN pt = transformed_cloud->at(i);

    bool is_collision =
        Pr2GripperModel::IsGripperFramePtInCollision(pt.x, pt.y, pt.z);
    bool is_grasp =
        Pr2GripperModel::IsGripperFramePtInGraspRegion(pt.x, pt.y, pt.z);
    if (!is_grasp && !is_collision) {
      continue;
    }

    bool is_antipodal = fabs(pt.normal_y) > kAntipodalCos;

    if (is_grasp && is_antipodal) {
      ++eval.features.antipodal_grasp_pts;
    } else if (is_grasp && !is_antipodal) {
      ++eval.features.non_antipodal_grasp_pts;
    } else if (!is_grasp && is_antipodal) {
      ++eval.features.antipodal_collisions;
    } else {
      ++eval.features.non_antipodal_collisions;
    }
  }

  eval.features.sq_wrist_distance =
      (model.palm_center() - wrist_pos).squaredNorm();

  eval.features.is_colliding_with_obstacle =
      IsGripperCollidingWithObstacles(model, context);
  return eval;
}

Pose GraspPlanner::OptimizePlacement(const Pose& gripper_pose,
                                     const GraspPlanningContext& context,
                                     int max_iters) {
  Pose current_pose = gripper_pose;

  Eigen::Affine3d affine_pose;
  tf::poseMsgToEigen(current_pose, affine_pose);
  Eigen::Vector3d gripper_center;  // In gripper frame
  gripper_center << Pr2GripperModel::kGraspRegionPos.x,
      Pr2GripperModel::kGraspRegionPos.y, Pr2GripperModel::kGraspRegionPos.z;

  int num_collisions = 0;
  for (int iter = 0; iter < max_iters; ++iter) {
    num_collisions = 0;
    PointCloudP::Ptr object_cloud = context.object_cloud();
    PointCloudP::Ptr transformed_cloud(new PointCloudP);
    pcl::transformPointCloud(*object_cloud, *transformed_cloud,
                             affine_pose.inverse());

    Eigen::Vector3d total = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < transformed_cloud->size(); ++i) {
      PointP pt = transformed_cloud->at(i);
      if (Pr2GripperModel::IsGripperFramePtInCollision(pt.x, pt.y, pt.z)) {
        ++num_collisions;
        Eigen::Vector3d pt_vec;
        pt_vec << pt.x, pt.y, pt.z;
        Eigen::Vector3d center_to_pt = pt_vec - gripper_center;
        total += 2 * center_to_pt;
      }
    }

    int num_pts_in_grasp = 0;
    for (size_t i = 0; i < transformed_cloud->size(); ++i) {
      PointP pt = transformed_cloud->at(i);
      if (Pr2GripperModel::IsGripperFramePtInGraspRegion(pt.x, pt.y, pt.z)) {
        Eigen::Vector3d pt_vec;
        pt_vec << pt.x, pt.y, pt.z;
        Eigen::Vector3d center_to_pt = pt_vec - gripper_center;
        center_to_pt.x() = 0;
        total += center_to_pt;
        ++num_pts_in_grasp;
      }
    }

    total /= (num_pts_in_grasp + num_collisions);

    if (total.norm() < 0.001) {
      break;
    }

    affine_pose.translate(total);
    tf::poseEigenToMsg(affine_pose, current_pose);

    if (debug_) {
      VisualizeGripper("optimization", current_pose,
                       context.planning_frame_id());
    }
  }

  return current_pose;
}

void GraspPlanner::UpdateParams() {
  debug_ = rapid::GetBoolParamOrThrow("grasp_planner/debug");
  weights_.antipodal_grasp_weight =
      GetDoubleParamOrThrow("grasp_planner/antipodal_grasp_weight");
  weights_.non_antipodal_grasp_weight =
      GetDoubleParamOrThrow("grasp_planner/non_antipodal_grasp_weight");
  weights_.antipodal_collision_weight =
      GetDoubleParamOrThrow("grasp_planner/antipodal_collision_weight");
  weights_.non_antipodal_collision_weight =
      GetDoubleParamOrThrow("grasp_planner/non_antipodal_collision_weight");
  weights_.sq_wrist_distance_weight =
      GetDoubleParamOrThrow("grasp_planner/sq_wrist_distance_weight");
  weights_.obstacle_collision_weight =
      GetDoubleParamOrThrow("grasp_planner/obstacle_collision_weight");

  const double kAntipodalDegrees =
      GetDoubleParamOrThrow("grasp_planner/antipodal_degrees");
  kAntipodalCos = cos(kAntipodalDegrees * M_PI / 180);
}

bool IsGripperCollidingWithObstacles(const Pr2GripperModel& gripper,
                                     const GraspPlanningContext& context) {
  const std::vector<Obb>& obstacles = context.obstacles();
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const Obb& obstacle = obstacles[i];
    if (gripper.IsCollidingWithObb(obstacle.pose, obstacle.dims)) {
      return true;
    }
  }
  return false;
}

int NumCollisions(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr obj_in_gripper) {
  int num_collisions = 0;
  for (size_t i = 0; i < obj_in_gripper->size(); ++i) {
    const PointP& pt = obj_in_gripper->at(i);
    if (Pr2GripperModel::IsGripperFramePtInCollision(pt.x, pt.y, pt.z)) {
      ++num_collisions;
    }
  }
  return num_collisions;
}

int NumPointsInGraspRegion(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr obj_in_gripper) {
  int num_pts = 0;
  for (size_t i = 0; i < obj_in_gripper->size(); ++i) {
    const PointP& pt = obj_in_gripper->at(i);
    if (Pr2GripperModel::IsGripperFramePtInGraspRegion(pt.x, pt.y, pt.z)) {
      ++num_pts;
    }
  }
  return num_pts;
}
}  // namespace pbi
