#include "task_imitation/grasp_planner.h"

#include <limits.h>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "pcl/common/transforms.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "task_perception/pcl_typedefs.h"
#include "task_perception/pcl_utils.h"
#include "task_perception/pr2_gripper_model.h"
#include "task_perception_msgs/DemoState.h"
#include "task_utils/ros_params.h"
#include "transform_graph/graph.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_imitation/grasp_planning_context.h"

namespace tg = transform_graph;
namespace msgs = task_perception_msgs;
using geometry_msgs::Pose;

namespace pbi {
GraspFeatures::GraspFeatures()
    : antipodal_grasp_pts(0),
      non_antipodal_grasp_pts(0),
      antipodal_collisions(0),
      non_antipodal_collisions(0),
      sq_wrist_distance(0) {}

GraspFeatureWeights::GraspFeatureWeights()
    : antipodal_grasp_weight(0),
      non_antipodal_grasp_weight(0),
      antipodal_collision_weight(0),
      non_antipodal_collision_weight(0),
      sq_wrist_distance_weight(0) {}

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
  return score;
}

GraspPlanner::GraspPlanner()
    : nh_(),
      gripper_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "grasp_planner/grippers", 1, true)),
      object_pub_(nh_.advertise<sensor_msgs::PointCloud2>(
          "grasp_planner/object", 1, true)),
      kGripperMarkers(),
      debug_(false) {
  InitGripperMarkers();
}

Pose GraspPlanner::Plan(const GraspPlanningContext& context) {
  UpdateParams();
  Pose wrist_pose = context.wrist_pose();
  Pr2GripperModel gripper_model;
  gripper_model.set_pose(wrist_pose);
  if (debug_) {
    VisualizeGripper("optimization", wrist_pose, context.planning_frame_id());
    ros::Duration(0.2).sleep();
  }

  PublishPointCloud(object_pub_, *context.object_cloud());

  Pose initial_pose = ComputeInitialGrasp(gripper_model, context);
  gripper_model.set_pose(initial_pose);
  if (debug_) {
    VisualizeGripper("optimization", initial_pose, context.planning_frame_id());
    ros::Duration(0.2).sleep();
  }

  Pose to_wrist_pose = OrientTowardsWrist(gripper_model, context);
  gripper_model.set_pose(to_wrist_pose);
  if (debug_) {
    VisualizeGripper("optimization", to_wrist_pose,
                     context.planning_frame_id());
    ros::Duration(0.2).sleep();
  }

  return Plan(to_wrist_pose, context);
}

Pose GraspPlanner::Plan(const Pose& initial_pose,
                        const GraspPlanningContext& context) {
  UpdateParams();
  Pr2GripperModel gripper_model;
  gripper_model.set_pose(initial_pose);

  PublishPointCloud(object_pub_, *context.object_cloud());

  // Get current wrist pose and previous gripper pose
  tg::Graph graph;

  const double kTranslationThreshold = 0.015;
  const double kRotationThreshold = 0.1;

  // While termination criteria not reached:
  // - Point towards wrist
  // - Optimize orientation
  // - Optimize position
  // Check for collisions
  Pose prev_pose = initial_pose;
  Pose next_pose;
  for (int i = 0; i < 5; ++i) {
    Pose placed = OptimizePlacement(prev_pose, context, 10);
    gripper_model.set_pose(placed);
    if (debug_) {
      VisualizeGripper("optimization", placed, context.planning_frame_id());
      ros::Duration(0.2).sleep();
    }

    Pose rotated_pose = OptimizeOrientation(gripper_model, context);
    gripper_model.set_pose(rotated_pose);
    if (debug_) {
      VisualizeGripper("optimization", rotated_pose,
                       context.planning_frame_id());
      ros::Duration(0.20).sleep();
    }

    next_pose = rotated_pose;

    Eigen::Affine3d prev_affine;
    tf::poseMsgToEigen(prev_pose, prev_affine);
    Eigen::Affine3d next_affine;
    tf::poseMsgToEigen(next_pose, next_affine);
    double translation =
        (next_affine.translation() - prev_affine.translation()).norm();

    if (translation < kTranslationThreshold) {
      Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
          prev_affine.rotation().col(1), next_affine.rotation().col(1));
      Eigen::AngleAxisd aa(rotation);
      double angle = aa.angle();
      if (angle < kRotationThreshold) {
        break;
      } else {
        ROS_INFO("Rotated by %f, running again", angle);
      }
    } else {
      ROS_INFO("Moved by %f, running again", translation);
    }
    prev_pose = next_pose;
  }
  if (debug_) {
    VisualizeGripper("optimization", next_pose, context.planning_frame_id());
  }
  return next_pose;
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

  Eigen::Affine3d grasp_pose;
  tf::poseMsgToEigen(gripper_model.pose(), grasp_pose);

  const geometry_msgs::Pose& wrist_pose = context.wrist_pose();
  Eigen::Vector3d wrist_pos;
  wrist_pos << wrist_pose.position.x, wrist_pose.position.y,
      wrist_pose.position.z;

  Pose best_pose;
  double best_score = -std::numeric_limits<double>::max();

  const double kYawRange = 120 * M_PI / 180;
  const double kYawResolution = 12 * M_PI / 180;
  int num_yaw_samples = round(kYawRange / kYawResolution) + 1;
  for (int yaw_i = 0; yaw_i < num_yaw_samples; ++yaw_i) {
    double yaw_angle = yaw_i * kYawResolution - kYawRange / 2;
    Eigen::AngleAxisd yaw_rot(yaw_angle, Eigen::Vector3d::UnitZ());

    const double kRollRange = 90 * M_PI / 180;
    const double kRollResolution = 10 * M_PI / 180;
    int num_roll_samples = round(kRollRange / kRollResolution) + 1;
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

      GraspEvaluation grasp_eval = ScoreGrasp(rotated_mat, wrist_pos, context);
      double score = grasp_eval.score();

      if (score > best_score) {
        best_score = score;
        best_pose = rotated_pose;
        if (debug_) {
          VisualizeGripper("optimization", rotated_pose,
                           context.planning_frame_id());
          ros::Duration(0.01).sleep();
          // ROS_INFO("%f %f: %s", yaw_angle, roll_angle,
          //         grasp_eval.ToString().c_str());
        }
      } else {
        if (debug_) {
          // VisualizeGripper("optimization", rotated_pose,
          //                 context.planning_frame_id());
          // ros::Duration(0.01).sleep();
        }
      }
    }
  }

  const double kPitchRange = 63 * M_PI / 180;
  const double kPitchResolution = 9 * M_PI / 180;
  int num_pitch_samples = round(kPitchRange / kPitchResolution) + 1;
  for (int pitch_i = 0; pitch_i < num_pitch_samples; ++pitch_i) {
    double pitch_angle = pitch_i * kPitchResolution - kPitchRange / 2;
    Eigen::AngleAxisd pitch_rot(pitch_angle, Eigen::Vector3d::UnitY());
    tf_graph.Add("rotated grasp center", tg::RefFrame("grasp center"),
                 tg::Transform(tg::Position(), pitch_rot.matrix()));

    // New gripper pose in camera frame, after rotation.
    tg::Transform rotated_tf;
    tf_graph.DescribePose(gripper_in_grasp_center,
                          tg::Source("rotated grasp center"),
                          tg::Target("gripper base"), &rotated_tf);
    Pose rotated_pose;
    rotated_tf.ToPose(&rotated_pose);
    Eigen::Affine3d rotated_mat(rotated_tf.matrix());

    GraspEvaluation grasp_eval;
    ScoreGrasp(rotated_mat, wrist_pos, context);

    if (debug_) {
      // ROS_INFO("p: %f, %s", pitch_angle * 180 / M_PI,
      //         grasp_eval.ToString().c_str());
      VisualizeGripper("optimization", rotated_pose,
                       context.planning_frame_id());
      ros::Duration(0.025).sleep();
    }

    double score = grasp_eval.score();
    if (score > best_score) {
      best_score = score;
      best_pose = rotated_pose;
    }
  }
  return best_pose;
}

GraspEvaluation GraspPlanner::ScoreGrasp(const Eigen::Affine3d& gripper_pose,
                                         const Eigen::Vector3d& wrist_pos,
                                         const GraspPlanningContext& context) {
  GraspEvaluation eval;
  const double kAntipodalDegrees = 15;
  const double kAntipodalCos = cos(kAntipodalDegrees * M_PI / 180);
  eval.weights = weights_;

  // Transform object points into rotated gripper frame.
  PointCloudN::Ptr object_cloud = context.object_cloud_with_normals();
  PointCloudN::Ptr transformed_cloud(new PointCloudN);
  pcl::transformPointCloudWithNormals(*object_cloud, *transformed_cloud,
                                      gripper_pose.inverse());

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
      (gripper_pose.translation() - wrist_pos).squaredNorm();
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

  for (int iter = 0; iter < max_iters; ++iter) {
    PointCloudP::Ptr object_cloud = context.object_cloud();
    PointCloudP::Ptr transformed_cloud(new PointCloudP);
    pcl::transformPointCloud(*object_cloud, *transformed_cloud,
                             affine_pose.inverse());

    Eigen::Vector3d total = Eigen::Vector3d::Zero();

    // Number of points in collision
    int num_collisions = 0;
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
        total += center_to_pt;
        ++num_pts_in_grasp;
      }
    }

    total /= (num_pts_in_grasp + num_collisions);

    if (total.norm() < 0.001) {
      return current_pose;
    }

    affine_pose.translate(total);
    tf::poseEigenToMsg(affine_pose, current_pose);

    if (debug_) {
      VisualizeGripper("optimization", current_pose,
                       context.planning_frame_id());
      ros::Duration(0.01).sleep();
    }
  }
  return current_pose;
}

void GraspPlanner::UpdateParams() {
  ros::param::get("grasp_planner/debug", debug_);
  GetParam("grasp_planner/antipodal_grasp_weight",
           &weights_.antipodal_grasp_weight);
  GetParam("grasp_planner/non_antipodal_grasp_weight",
           &weights_.non_antipodal_grasp_weight);
  GetParam("grasp_planner/antipodal_collision_weight",
           &weights_.antipodal_collision_weight);
  GetParam("grasp_planner/non_antipodal_collision_weight",
           &weights_.non_antipodal_collision_weight);
  GetParam("grasp_planner/sq_wrist_distance_weight",
           &weights_.sq_wrist_distance_weight);
}
}  // namespace pbi
