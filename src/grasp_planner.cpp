#include "task_perception/grasp_planner.h"

#include <limits.h>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "pcl/common/transforms.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "transform_graph/graph.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_perception/pcl_typedefs.h"
#include "task_perception/pr2_gripper_model.h"
#include "task_perception/task_perception_context.h"

namespace tg = transform_graph;
using geometry_msgs::Pose;

namespace pbi {
GraspPlanner::GraspPlanner()
    : nh_(),
      gripper_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "grasp_planner/grippers", 1, true)),
      kGripperMarkers(),
      kDebug_(true) {
  InitGripperMarkers();
}

void GraspPlanner::Plan(const std::string& left_or_right,
                        const std::string& object_name,
                        TaskPerceptionContext* context, Pose* pose) {
  Pose wrist_pose;
  if (left_or_right == "left") {
    wrist_pose = context->GetLeftWristPose();
  } else {
    wrist_pose = context->GetRightWristPose();
  }
  Pr2GripperModel gripper_model;
  gripper_model.set_pose(wrist_pose);

  Pose initial_pose;
  ComputeInitialGrasp(gripper_model, object_name, context, &initial_pose);
  gripper_model.set_pose(initial_pose);
  if (kDebug_) {
    VisualizeGripper("optimization", initial_pose,
                     context->camera_info().header.frame_id);
    ros::Duration(0.25).sleep();
  }

  Pose next_pose;
  OrientTowardsWrist(gripper_model, wrist_pose, context, &next_pose);
  gripper_model.set_pose(next_pose);
  if (kDebug_) {
    VisualizeGripper("optimization", next_pose,
                     context->camera_info().header.frame_id);
    ros::Duration(0.25).sleep();
  }

  Plan(left_or_right, object_name, initial_pose, context, pose);
}

void GraspPlanner::Plan(const std::string& left_or_right,
                        const std::string& object_name,
                        const Pose& initial_pose,
                        TaskPerceptionContext* context, Pose* pose) {
  Pr2GripperModel gripper_model;
  gripper_model.set_pose(initial_pose);

  Pose wrist_pose;
  if (left_or_right == "left") {
    wrist_pose = context->GetLeftWristPose();
  } else {
    wrist_pose = context->GetRightWristPose();
  }

  const double kTranslationThreshold = 0.001;
  const double kRotationThreshold = 0.01;

  // While termination criteria not reached:
  // - Point towards wrist
  // - Optimize orientation
  // - Optimize position
  // Check for collisions
  Pose prev_pose = initial_pose;
  Pose next_pose;
  for (int i = 0; i < 5; ++i) {
    ROS_INFO("Iteration %d", i + 1);

    OptimizeOrientation(gripper_model, object_name, wrist_pose, context,
                        &next_pose);
    gripper_model.set_pose(next_pose);
    if (kDebug_) {
      VisualizeGripper("optimization", next_pose,
                       context->camera_info().header.frame_id);
      ros::Duration(0.25).sleep();
    }

    OptimizePlacement(gripper_model, object_name, context, &next_pose);
    gripper_model.set_pose(next_pose);
    if (kDebug_) {
      VisualizeGripper("optimization", next_pose,
                       context->camera_info().header.frame_id);
      ros::Duration(0.25).sleep();
    }

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
  VisualizeGripper("optimization", next_pose,
                   context->camera_info().header.frame_id);
  *pose = next_pose;
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

void GraspPlanner::ComputeInitialGrasp(const Pr2GripperModel& gripper_model,
                                       const std::string& object_name,
                                       TaskPerceptionContext* context,
                                       Pose* initial_pose) {
  Pose wrist_pose = gripper_model.pose();

  // Initial pose to optimize around: translate gripper such that the closest
  // object point is in the center.
  KdTreeP::Ptr object_tree = context->GetObjectTree(object_name);
  PointP wrist_point;
  wrist_point.x = wrist_pose.position.x;
  wrist_point.y = wrist_pose.position.y;
  wrist_point.z = wrist_pose.position.z;
  std::vector<int> indices;
  std::vector<float> sq_distances;
  object_tree->nearestKSearch(wrist_point, 1, indices, sq_distances);
  PointP nearest_obj_pt = object_tree->getInputCloud()->at(indices[0]);

  Eigen::Vector3d nearest_obj_vec;
  nearest_obj_vec << nearest_obj_pt.x, nearest_obj_pt.y, nearest_obj_pt.z;
  Eigen::Vector3d translation = nearest_obj_vec - gripper_model.grasp_center();

  *initial_pose = wrist_pose;
  initial_pose->position.x += translation.x();
  initial_pose->position.y += translation.y();
  initial_pose->position.z += translation.z();
}

void GraspPlanner::OrientTowardsWrist(const Pr2GripperModel& gripper_model,
                                      const geometry_msgs::Pose& wrist_pose,
                                      TaskPerceptionContext* context,
                                      geometry_msgs::Pose* next_pose) {
  // Rotate the gripper about the center of the grasp region (CoG) such that the
  // centerline of the gripper is coincident with the line between the wrist and
  // the CoG.
  tg::Graph tf_graph = gripper_model.tf_graph();

  // Vector from CoG to wrist
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
  out.ToPose(next_pose);
}

void GraspPlanner::OptimizeOrientation(const Pr2GripperModel& gripper_model,
                                       const std::string& object_name,
                                       const Pose& wrist_pose,
                                       TaskPerceptionContext* context,
                                       Pose* next_pose) {
  // Parameters
  const int kNumSamplesPerDim = 10;

  // Sample roll and yaw angles (about the center of the grasp)
  // Get transform describing the gripper in the grasp center and cache it.
  tg::Graph tf_graph = gripper_model.tf_graph();
  tg::Transform gripper_in_grasp_center;
  tf_graph.ComputeDescription(tg::LocalFrame("gripper"),
                              tg::RefFrame("grasp center"),
                              &gripper_in_grasp_center);

  // Cache the transform from the camera to the grasp center, which will
  // eliminate a matrix multiplication in the inner loop.
  tg::Transform gripper_in_camera;
  tf_graph.ComputeDescription(tg::LocalFrame("grasp center"),
                              tg::RefFrame("gripper base"), &gripper_in_camera);
  tf_graph.Add("grasp center", tg::RefFrame("gripper base"), gripper_in_camera);

  Eigen::Affine3d grasp_pose;
  tf::poseMsgToEigen(gripper_model.pose(), grasp_pose);

  Eigen::Vector3d wrist_pos;
  wrist_pos << wrist_pose.position.x, wrist_pose.position.y,
      wrist_pose.position.z;

  Pose best_pose;
  double best_score = -std::numeric_limits<double>::max();

  const double kRangeDegs = 120;
  const double kRangeRads = M_PI * kRangeDegs / 180;
  for (int yaw_i = 0; yaw_i < kNumSamplesPerDim; ++yaw_i) {
    double yaw_angle = yaw_i * kRangeRads / kNumSamplesPerDim - kRangeRads / 2;
    Eigen::AngleAxisd yaw_rot(yaw_angle, Eigen::Vector3d::UnitZ());
    for (int roll_i = 0; roll_i < kNumSamplesPerDim; ++roll_i) {
      double roll_angle = roll_i * (M_PI / 2) / kNumSamplesPerDim - (M_PI / 4);
      // Compute rotation about the grasp center.
      Eigen::AngleAxisd roll_rot(roll_angle, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond rotation = roll_rot * yaw_rot;

      tg::Transform grasp_center_in_camera;
      tf_graph.ComputeDescription(tg::LocalFrame("grasp center"),
                                  tg::RefFrame("gripper base"),
                                  &grasp_center_in_camera);

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

      double score = ScoreGrasp(rotated_mat, object_name, wrist_pos, context);

      if (score > best_score) {
        best_score = score;
        best_pose = rotated_pose;
        if (kDebug_) {
          // int num_pts_in_grasp =
          //    num_antipodal_grasps + num_non_antipodal_grasps;
          // int num_antipodal_pts =
          //    num_antipodal_grasps + num_antipodal_collisions;
          // int num_collisions =
          //    num_antipodal_collisions + num_non_antipodal_collisions;
          /*ROS_INFO(
              "Best pose so far: y: %f, r: %f (%d pts in grasp, %d antipodal, "
              "%d collisions, dist: %f), score: %f",
              yaw_angle * 180 / M_PI, roll_angle * 180 / M_PI, num_pts_in_grasp,
              num_antipodal_pts, num_collisions, sqrt(sq_wrist_distance),
              score);*/

          VisualizeGripper("optimization", rotated_pose,
                           context->camera_info().header.frame_id);
          ros::Duration(0.01).sleep();
          // ros::topic::waitForMessage<std_msgs::Bool>("trigger");
        }
      } else {
        if (kDebug_) {
          VisualizeGripper("optimization", rotated_pose,
                           context->camera_info().header.frame_id);
          // ros::Duration(0.01).sleep();
        }
      }
    }
  }
  *next_pose = best_pose;
}

double GraspPlanner::ScoreGrasp(const Eigen::Affine3d& gripper_pose,
                                const std::string& object_name,
                                const Eigen::Vector3d& wrist_pos,
                                TaskPerceptionContext* context) {
  const double kAntipodalDegrees = 15;
  const double kAntipodalCos = cos(kAntipodalDegrees * M_PI / 180);

  // Transform object points into rotated gripper frame.
  PointCloudN::Ptr object_cloud =
      context->GetObjectCloudWithNormals(object_name);
  PointCloudN::Ptr transformed_cloud(new PointCloudN);
  pcl::transformPointCloudWithNormals(*object_cloud, *transformed_cloud,
                                      gripper_pose.inverse());

  // For all points in the grasp region, measure antipodality (i.e., how
  // collinear the normal is with the finger normals). Instead of
  // averaging, we count the number of normals that are antipodal "enough."
  int num_antipodal_grasps = 0;
  int num_non_antipodal_grasps = 0;
  int num_antipodal_collisions = 0;
  int num_non_antipodal_collisions = 0;
  double score = 0;

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
      ++num_antipodal_grasps;
    } else if (is_grasp && !is_antipodal) {
      ++num_non_antipodal_grasps;
    } else if (!is_grasp && is_antipodal) {
      ++num_antipodal_collisions;
    } else {
      ++num_non_antipodal_collisions;
    }
  }

  double sq_wrist_distance =
      (gripper_pose.translation() - wrist_pos).squaredNorm();

  score = 5 * num_antipodal_grasps + num_non_antipodal_grasps +
          0 * num_antipodal_collisions - 1 * num_non_antipodal_collisions -
          750 * sq_wrist_distance;

  return score;
}

void GraspPlanner::OptimizePlacement(const Pr2GripperModel& gripper_model,
                                     const std::string& object_name,
                                     TaskPerceptionContext* context,
                                     geometry_msgs::Pose* next_pose) {
  Pose current_pose = gripper_model.pose();

  Eigen::Affine3d affine_pose;
  tf::poseMsgToEigen(current_pose, affine_pose);
  Eigen::Vector3d gripper_center;  // In gripper frame
  gripper_center << Pr2GripperModel::kGraspRegionPos.x,
      Pr2GripperModel::kGraspRegionPos.y, Pr2GripperModel::kGraspRegionPos.z;

  for (int iter = 0; iter < 10; ++iter) {
    PointCloudP::Ptr object_cloud = context->GetObjectCloud(object_name);
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

    total /= num_pts_in_grasp;

    if (total.norm() < 0.001) {
      tf::poseEigenToMsg(affine_pose, *next_pose);
      return;
    }

    affine_pose.translate(total);

    tf::poseEigenToMsg(affine_pose, current_pose);

    if (kDebug_) {
      VisualizeGripper("optimization", current_pose,
                       context->camera_info().header.frame_id);
      ros::Duration(0.05).sleep();
    }
  }
}
}  // namespace pbi
