#include "task_imitation/grasp_planner.h"

#include <limits.h>
#include <math.h>
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "boost/foreach.hpp"
#include "eigen_conversions/eigen_msg.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_state/conversions.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_manipulation/moveit_error_code.h"
#include "rapid_ros/params.h"
#include "rapid_utils/vector3.hpp"
#include "rapid_viz/axes_markers.h"
#include "robot_markers/builder.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "task_perception/pcl_typedefs.h"
#include "task_perception/pcl_utils.h"
#include "transform_graph/graph.h"
#include "urdf/model.h"

#include "task_imitation/ik.h"
#include "task_imitation/motion_planning.h"
#include "task_imitation/program_constants.h"

namespace tg = transform_graph;
using geometry_msgs::Pose;
using rapid::GetDoubleParamOrThrow;

namespace pbi {
GraspFeatures::GraspFeatures()
    : antipodal_grasp_pts(0),
      non_antipodal_grasp_pts(0),
      antipodal_collisions(0),
      non_antipodal_collisions(0),
      sq_wrist_distance(0),
      num_obstacle_collisions(0),
      future_pose_ratio(0) {}

GraspFeatureWeights::GraspFeatureWeights()
    : antipodal_grasp_weight(0),
      non_antipodal_grasp_weight(0),
      antipodal_collision_weight(0),
      non_antipodal_collision_weight(0),
      sq_wrist_distance_weight(0),
      obstacle_collision_weight(0),
      future_pose_weight(0) {}

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
     << " + " << weights.future_pose_weight << "*" << features.future_pose_ratio
     << " = " << weights.antipodal_grasp_weight * features.antipodal_grasp_pts
     << " + "
     << weights.non_antipodal_grasp_weight * features.non_antipodal_grasp_pts
     << " + "
     << weights.antipodal_collision_weight * features.antipodal_collisions
     << " + "
     << weights.non_antipodal_collision_weight *
            features.non_antipodal_collisions
     << " + " << weights.sq_wrist_distance_weight * features.sq_wrist_distance
     << " + " << weights.future_pose_weight * features.future_pose_ratio;
  if (features.num_obstacle_collisions > 1) {
    ss << " (" << features.num_obstacle_collisions << " collisions)";
  } else if (features.num_obstacle_collisions == 1) {
    ss << " (" << features.num_obstacle_collisions << " collision)";
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
  score += weights.future_pose_weight * features.future_pose_ratio;
  score += weights.obstacle_collision_weight * features.num_obstacle_collisions;
  return score;
}

ScoredGrasp::ScoredGrasp()
    : score(-std::numeric_limits<double>::max()), pose() {}

bool ScoredGrasp::IsValid() {
  return score != -std::numeric_limits<double>::max();
}

GraspPlanner::GraspPlanner(const Pr2GripperViz& gripper_viz)
    : nh_(),
      gripper_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "grasp_planner/grippers", 20, true)),
      object_pub_(nh_.advertise<sensor_msgs::PointCloud2>(
          "grasp_planner/object", 1, true)),
      gripper_viz_(gripper_viz),
      debug_(false),
      debug_cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>(
          "grasp_planner/cloud", 1, true)),
      evals_(),
      labels_() {}

Pose GraspPlanner::Plan(const GraspPlanningContext& context) {
  UpdateParams();
  Pose wrist_pose = context.wrist_pose();
  // if (debug_) {
  //}
  visualization_msgs::MarkerArray wrist_axes = rapid::AxesMarkerArray(
      "wrist", context.planning_frame_id(), wrist_pose, 0.1);
  gripper_pub_.publish(wrist_axes);

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
  // if (debug_) {
  //  ROS_INFO("Initial grasp");
  //  ros::topic::waitForMessage<std_msgs::Bool>("trigger");
  //}

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
  visualization_msgs::MarkerArray wrist_axes = rapid::AxesMarkerArray(
      "wrist", context.planning_frame_id(), context.wrist_pose(), 0.1);
  while (ros::ok() && gripper_pub_.getNumSubscribers() == 0) {
    ROS_WARN_THROTTLE(1, "Waiting for Rviz to subscribe to gripper markers...");
    ros::Duration(0.1).sleep();
  }
  gripper_pub_.publish(wrist_axes);

  // Visualize obstacles
  // const std::vector<Obb>& obstacles = context.obstacles();
  // visualization_msgs::MarkerArray marker_arr;
  // for (size_t i = 0; i < obstacles.size(); ++i) {
  //  const Obb& obb = obstacles[i];
  //  visualization_msgs::Marker box;
  //  box.ns = "obstacles";
  //  box.id = i;
  //  box.header.frame_id = context.planning_frame_id();
  //  box.type = box.CUBE;
  //  box.pose = obb.pose;
  //  box.scale = obb.dims;
  //  box.color.r = 1;
  //  box.color.g = 1;
  //  box.color.b = 1;
  //  box.color.a = 0.25;
  //  marker_arr.markers.push_back(box);
  //}
  // gripper_pub_.publish(marker_arr);

  // Sample some points
  const double leaf_size =
      rapid::GetDoubleParamOrThrow("grasp_planner/leaf_size");
  boost::shared_ptr<std::vector<int> > sample_indices(new std::vector<int>());
  *sample_indices = SampleObject(context, leaf_size);
  int num_samples = sample_indices->size();
  PublishPointCloud(debug_cloud_pub_, context.object_cloud(), sample_indices);
  ROS_INFO("Sampled %d points", num_samples);

  ScoredGrasp best;
  for (int i = num_samples - 1; i >= 0; --i) {
    const int sample_index = sample_indices->at(i);

    // Visualize the object normal
    VisualizePointNormal(context, sample_index);

    Pr2GripperModel model;
    model.set_pose(gripper_model.pose());

    Pose centered = CenterGraspOnPoint(model, context, sample_index);
    model.set_pose(centered);

    Pose aligned = AlignGraspWithPoint(model, context, sample_index);
    model.set_pose(aligned);
    Pose pitched_to_wrist = PitchToWrist(model, context);
    VisualizeGripper("optimization", pitched_to_wrist,
                     context.planning_frame_id());
    if (debug_) {
      // ROS_INFO("%d of %d: Aligned with normal and pitched to wrist",
      //         num_samples - i, num_samples);
      // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
    }

    Pose placed =
        OptimizePlacement(pitched_to_wrist, context, kMaxPlacementIterations);

    // Shift the gripper based on forward grasp center
    double obj_width = ComputeObjWidthInGraspRegion(placed, context);
    model.set_pose(placed);
    Eigen::Vector3d forward_grasp_center =
        model.forward_grasp_center(obj_width);
    Eigen::Vector3d forward_shift = model.grasp_center() - forward_grasp_center;
    Eigen::Affine3d affine_pose;
    tf::poseMsgToEigen(placed, affine_pose);
    affine_pose.pretranslate(forward_shift);
    tf::poseEigenToMsg(affine_pose, placed);
    VisualizeGripper("optimization", placed, context.planning_frame_id());
    if (debug_) {
      // ROS_INFO("%d of %d: Placed", num_samples - i, num_samples);
      // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
    }

    model.set_pose(placed);
    ScoredGrasp grasp = OptimizePitch(model, obj_width, context);
    VisualizeGripper("optimization", grasp.pose, context.planning_frame_id());
    // if (debug_) {
    //  ROS_INFO("Pitched");
    //  ros::topic::waitForMessage<std_msgs::Bool>("trigger");
    //}

    grasp.pose = MaximizeMargin(grasp.pose, context);
    VisualizeGripper("optimization", grasp.pose, context.planning_frame_id());
    if (debug_) {
      // ROS_INFO("%d of %d: Pitched and centered", num_samples - i,
      // num_samples); ros::topic::waitForMessage<std_msgs::Bool>("trigger");
    }

    model.set_pose(grasp.pose);
    // if (NumObstacleCollisions(model, context) > 0) {
    //  grasp = EscapeCollision(model, context);
    //  VisualizeGripper("optimization", grasp.pose,
    //  context.planning_frame_id()); if (debug_) {
    //    ROS_INFO("Escape attempt");
    //    ros::topic::waitForMessage<std_msgs::Bool>("trigger");
    //  }
    //}

    if (grasp.IsValid()) {
      // If we can possibly do better by reaching more poses, evaluate future
      // poses.
      int num_future_poses = 0;
      if (grasp.score + grasp.eval.weights.future_pose_weight > best.score) {
        model.set_pose(grasp.pose);
        num_future_poses = EvaluateFuturePoses(model, context);
        grasp.eval.features.future_pose_ratio =
            static_cast<double>(num_future_poses) /
            context.future_poses().size();
        grasp.score = grasp.eval.score();
      }

      if (grasp.score > best.score) {
        bool found_plan = IsGraspReachable(*context.move_group(), grasp.pose);
        if (!found_plan) {
          ROS_INFO("%d of %d: Skipping unreachable %s", num_samples - i,
                   num_samples, grasp.eval.ToString().c_str());
        } else {
          ROS_INFO("%d of %d: Best score: %s", num_samples - i, num_samples,
                   grasp.eval.ToString().c_str());
          ROS_INFO(
              "%d of %d: Adopting grasp that reaches %d of %zu future poses",
              num_samples - i, num_samples, num_future_poses,
              context.future_poses().size());

          best = grasp;
        }
      } else {
        if (debug_) {
          ROS_INFO("%d of %d: Eval: %s", num_samples - i, num_samples,
                   grasp.eval.ToString().c_str());
        }
      }
      if (debug_) {
        evals_.push_back(grasp.eval);
      }
    }
    VisualizeGripper("optimization_best", best.pose,
                     context.planning_frame_id());
    if (debug_) {
      // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
      std::string label;
      std::cout << "Label 0 or 1: ";
      std::getline(std::cin, label);
      labels_.push_back(atoi(label.c_str()));
    }
  }
  VisualizeGripper("optimization_best", best.pose, context.planning_frame_id());
  if (best.IsValid()) {
    for (size_t i = 0; i < evals_.size(); ++i) {
      std::cout << evals_[i].ToString() << "," << labels_[i] << std::endl;
    }
    ROS_INFO("Planned grasp with score: %f", best.eval.score());
    return best.pose;
  } else {
    ROS_ERROR("Unable to plan grasp");
    Pose blank;
    return blank;
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

  visualization_msgs::MarkerArray meshes =
      gripper_viz_.markers(ns, pose, frame_id);
  marker_arr.markers.insert(marker_arr.markers.end(), meshes.markers.begin(),
                            meshes.markers.end());

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
                                            const double leaf_size) {
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud(context.object_cloud());
  Eigen::Vector4f leaf;
  leaf << leaf_size, leaf_size, leaf_size, 0;
  vox.setLeafSize(leaf);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(
      new pcl::PointCloud<pcl::PointXYZ>);
  vox.filter(*downsampled);

  std::vector<int> indices;
  for (size_t i = 0; i < downsampled->size(); i++) {
    const pcl::PointXYZ& pt = downsampled->at(i);
    std::vector<int> k_indices(1);
    std::vector<float> k_sq_dists(1);
    int num_neighbors =
        context.object_tree()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
    if (num_neighbors > 0) {
      indices.push_back(k_indices[0]);
    }
  }
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

Pose GraspPlanner::PitchToWrist(const Pr2GripperModel& gripper_model,
                                const GraspPlanningContext& context) {
  // Pitch the gripper about the center of the grasp region (CoG) such that the
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
  // Project the wrist pose onto the xz plane of the gripper
  Eigen::Vector3d desired_line = wrist_in_grasp_center.vector();
  desired_line.y() = 0;

  // Centerline
  tg::Transform gripper_in_grasp_center;
  tf_graph.ComputeDescription("gripper", tg::RefFrame("grasp center"),
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

ScoredGrasp GraspPlanner::OptimizePitch(const Pr2GripperModel& gripper_model,
                                        const double obj_width,
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
  Eigen::Vector3d forward_grasp_center =
      gripper_model.forward_grasp_center(obj_width);
  double forward_offset =
      (forward_grasp_center - gripper_model.grasp_center()).norm();
  tf_graph.Add(
      "forward grasp center", tg::RefFrame("grasp center"),
      tg::Transform(tg::Position(forward_offset, 0, 0), tg::Orientation()));
  tg::Transform gripper_in_grasp_center;
  tf_graph.ComputeDescription("gripper", tg::RefFrame("forward grasp center"),
                              &gripper_in_grasp_center);

  ScoredGrasp best;
  for (int pitch_i = 0; pitch_i < num_pitch_samples; ++pitch_i) {
    double pitch_angle = pitch_i * pitch_resolution - half_pitch_range;
    Eigen::AngleAxisd pitch_rot(pitch_angle, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond rotation(pitch_rot);

    tf_graph.Add("rotated grasp center", tg::RefFrame("forward grasp center"),
                 tg::Transform(tg::Position(), rotation));
    // New gripper pose in camera frame, after rotation.
    tg::Transform rotated_tf;
    tf_graph.DescribePose(gripper_in_grasp_center,
                          tg::Source("rotated grasp center"),
                          tg::Target("gripper base"), &rotated_tf);
    Pose rotated_pose = rotated_tf.pose();

    Pr2GripperModel candidate;
    candidate.set_pose(rotated_pose);
    // if (debug_) {
    //  VisualizeGripper("optimization", rotated_pose,
    //                   context.planning_frame_id());
    //  ros::Duration(0.01).sleep();
    //}
    Eigen::Vector3d candidate_pos = rapid::AsVector3d(rotated_pose.position);
    if ((candidate_pos - wrist_pos).norm() > 0.4) {
      continue;
    }

    if (IsPalmCollidingWithAllObstacles(candidate, context)) {
      continue;
    }
    // Optimize soft constraints
    GraspEvaluation grasp_eval = ScoreGrasp(candidate, wrist_pos, context);
    double score = grasp_eval.score();

    // if (debug_) {
    //  ROS_INFO("p: %f: %s", pitch_angle * 180 / M_PI,
    //           grasp_eval.ToString().c_str());
    //}
    if (score > best.score) {
      best.score = score;
      best.eval = grasp_eval;
      best.pose = rotated_pose;
      // if (debug_) {
      //   ROS_INFO("p: %f: %s", pitch_angle * 180 / M_PI,
      //           grasp_eval.ToString().c_str());
      //}
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
  GraspEvaluation initial_eval = ScoreGrasp(gripper_model, wrist_pos, context);
  best.score = initial_eval.score();
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

    // If we are not colliding with anything, return.
    GraspEvaluation grasp_eval = ScoreGrasp(model, wrist_pos, context);
    double score = grasp_eval.score();

    if (score > best.score) {
      best.score = score;
      best.eval = grasp_eval;
      best.pose = moved_pose;
      // if (debug_) {
      //  ROS_INFO("[Escape] Best score so far: %s",
      //           grasp_eval.ToString().c_str());
      //}
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

  eval.features.num_obstacle_collisions = NumObstacleCollisions(model, context);
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

  double placement_step_scale =
      rapid::GetDoubleParamOrThrow("grasp_planner/placement_step_scale");

  int num_collisions = 0;
  double decay = 1;
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
        center_to_pt.x() = 0;
        center_to_pt.z() = 0;
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
        center_to_pt.z() = 0;
        total += center_to_pt;
        ++num_pts_in_grasp;
      }
    }

    total /= (num_pts_in_grasp + num_collisions);
    total *= decay;

    if (total.norm() < 0.001) {
      break;
    }

    affine_pose.translate(placement_step_scale * total);
    tf::poseEigenToMsg(affine_pose, current_pose);

    VisualizeGripper("optimization", current_pose, context.planning_frame_id());
    if (debug_) {
      // ros::Duration(0.05).sleep();
    }
    decay *= 0.9;
  }

  return current_pose;
}

Pose GraspPlanner::MaximizeMargin(const geometry_msgs::Pose& gripper_pose,
                                  const GraspPlanningContext& context) {
  Pose current_pose = gripper_pose;

  Eigen::Affine3d affine_pose;
  tf::poseMsgToEigen(current_pose, affine_pose);
  PointCloudP::Ptr object_cloud = context.object_cloud();
  PointCloudP::Ptr transformed_cloud(new PointCloudP);
  pcl::transformPointCloud(*object_cloud, *transformed_cloud,
                           affine_pose.inverse());

  double min_y = std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();
  for (size_t i = 0; i < transformed_cloud->size(); ++i) {
    PointP pt = transformed_cloud->at(i);
    if (Pr2GripperModel::IsGripperFramePtInGraspRegion(pt.x, pt.y, pt.z)) {
      if (pt.y < min_y) {
        min_y = pt.y;
      }
      if (pt.y > max_y) {
        max_y = pt.y;
      }
    }
  }

  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  translation.y() = (max_y + min_y) / 2;

  affine_pose.translate(translation);
  tf::poseEigenToMsg(affine_pose, current_pose);

  VisualizeGripper("optimization", current_pose, context.planning_frame_id());
  if (debug_) {
    // ros::Duration(0.05).sleep();
  }
  return current_pose;
}

double GraspPlanner::ComputeObjWidthInGraspRegion(
    const geometry_msgs::Pose& gripper_pose,
    const GraspPlanningContext& context) {
  double grasp_min_y = std::numeric_limits<double>::max();
  double grasp_max_y = -std::numeric_limits<double>::max();

  PointCloudP::Ptr object_cloud = context.object_cloud();
  PointCloudP::Ptr transformed_cloud(new PointCloudP);
  Eigen::Affine3d affine_pose;
  tf::poseMsgToEigen(gripper_pose, affine_pose);
  pcl::transformPointCloud(*object_cloud, *transformed_cloud,
                           affine_pose.inverse());

  for (size_t i = 0; i < transformed_cloud->size(); ++i) {
    PointP pt = transformed_cloud->at(i);
    if (Pr2GripperModel::IsGripperFramePtInGraspRegion(pt.x, pt.y, pt.z)) {
      if (pt.y < grasp_min_y) {
        grasp_min_y = pt.y;
      }
      if (pt.y > grasp_max_y) {
        grasp_max_y = pt.y;
      }
    }
  }
  return grasp_max_y - grasp_min_y;
}

int GraspPlanner::EvaluateFuturePoses(const Pr2GripperModel& model,
                                      const GraspPlanningContext& context) {
  tg::Graph graph;
  graph.Add("gripper", tg::RefFrame("planning"), model.pose());
  graph.Add("original object", tg::RefFrame("planning"), context.object_pose());
  tg::Transform grasp_in_obj;
  graph.ComputeDescription("gripper", tg::RefFrame("original object"),
                           &grasp_in_obj);

  const std::vector<TypedPose>& future_poses = context.future_poses();
  int count = 0;
  for (size_t i = 0; i < future_poses.size(); i++) {
    const TypedPose& typed_pose = future_poses[i];
    const Pose& obj_in_planning = typed_pose.pose;
    graph.Add("object", tg::RefFrame("planning"), obj_in_planning);
    Eigen::Affine3d obj_in_planning_affine;
    tf::poseMsgToEigen(obj_in_planning, obj_in_planning_affine);
    bool found_ik = false;
    for (int yaw_i = 0; yaw_i < 4; ++yaw_i) {
      double yaw_angle = yaw_i * M_PI / 2;
      Eigen::AngleAxisd yaw_rot(yaw_angle, Eigen::Vector3d::UnitZ());
      graph.Add("rotated object", tg::RefFrame("object"),
                tg::Transform(tg::Position(), Eigen::Quaterniond(yaw_rot)));
      tg::Transform rotated_gripper;
      graph.DescribePose(grasp_in_obj, tg::Source("rotated object"),
                         tg::Target("planning"), &rotated_gripper);
      Pose rotated_pose = rotated_gripper.pose();

      // if (debug_) {
      //  VisualizeGripper("optimization", rotated_pose,
      //                   context.planning_frame_id());
      //}

      Pr2GripperModel candidate;
      candidate.set_pose(rotated_pose);
      if (HasIk(*(context.move_group()), rotated_pose)) {
        ++count;
        found_ik = true;
        break;
      } else {
        if (debug_) {
          // ROS_INFO("Pose %zu yaw %f: No IK! %f %f %f %f %f %f %f", i,
          //         yaw_angle * 180 / M_PI, rotated_pose.position.x,
          //         rotated_pose.position.y, rotated_pose.position.z,
          //         rotated_pose.orientation.x, rotated_pose.orientation.y,
          //         rotated_pose.orientation.z, rotated_pose.orientation.w);
          // VisualizeGripper("optimization", rotated_pose,
          //                 context.planning_frame_id());
          // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
        }
        // Only try to rotate circular objects on trajectory or move-to steps
        bool can_rotate_step = typed_pose.type == TypedPose::MOVE_TO ||
                               typed_pose.type == TypedPose::TRAJECTORY;
        if (!context.IsObjectCircular() || !can_rotate_step) {
          break;
        }
      }
    }
    if (!found_ik) {
      break;
    }
  }
  return count;
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
  weights_.future_pose_weight =
      GetDoubleParamOrThrow("grasp_planner/future_pose_weight");

  const double kAntipodalDegrees =
      GetDoubleParamOrThrow("grasp_planner/antipodal_degrees");
  kAntipodalCos = cos(kAntipodalDegrees * M_PI / 180);
}

int NumObstacleCollisions(const Pr2GripperModel& gripper,
                          const GraspPlanningContext& context) {
  const std::vector<Obb>& obstacles = context.obstacles();
  int num_collisions = 0;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const Obb& obstacle = obstacles[i];
    if (gripper.IsCollidingWithObb(obstacle.pose, obstacle.dims)) {
      num_collisions++;
    }
  }
  return num_collisions;
}

bool IsPalmCollidingWithAllObstacles(const Pr2GripperModel& gripper,
                                     const GraspPlanningContext& context) {
  const std::vector<Obb>& obstacles = context.obstacles();
  size_t count = 0;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const Obb& obstacle = obstacles[i];
    if (gripper.CheckCollisionWithObb(obstacle.pose, obstacle.dims) ==
        Pr2GripperModel::PALM) {
      count++;
    }
  }
  return count == obstacles.size();
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

bool IsGraspReachable(moveit::planning_interface::MoveGroup& move_group,
                      const geometry_msgs::Pose& pose) {
  tg::Graph graph;
  graph.Add("pose", tg::RefFrame("planning"), pose);
  tg::Transform pregrasp_pose;
  graph.DescribePose(
      tg::Transform(tg::Position(-kPregraspDistance, 0, 0), tg::Orientation()),
      tg::Source("pose"), tg::Target("planning"), &pregrasp_pose);
  moveit::planning_interface::MoveGroup::Plan pregrasp_plan;
  std::string error = PlanToPose(move_group, *move_group.getCurrentState(),
                                 pregrasp_pose.pose(), 2, &pregrasp_plan);
  if (error != "") {
    return false;
  }

  moveit::core::RobotStatePtr state = move_group.getCurrentState();
  moveit::core::jointTrajPointToRobotState(
      pregrasp_plan.trajectory_.joint_trajectory,
      pregrasp_plan.trajectory_.joint_trajectory.points.size() - 1, *state);

  moveit::planning_interface::MoveGroup::Plan plan;
  error = PlanCartesianToPose(move_group, *state, pose, 10, &plan.trajectory_);
  return error == "";
}
}  // namespace pbi
