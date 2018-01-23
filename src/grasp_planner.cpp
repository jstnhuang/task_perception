#include "task_perception/grasp_planner.h"

#include <map>
#include <set>
#include <string>

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "transform_graph/graph.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_perception/task_perception_context.h"

namespace pbi {
GraspPlanner::GraspPlanner()
    : nh_(),
      gripper_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "grasp_planner/grippers", 1, true)),
      kGripperMarkers() {
  InitGripperMarkers();
}

void GraspPlanner::Plan(const std::string& left_or_right,
                        TaskPerceptionContext* context,
                        geometry_msgs::Pose* pose) {}

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
  builder.SetFrameId("head_mount_kinect_rgb_optical_frame");

  std::set<std::string> gripper_links;
  gripper_links.insert("l_gripper_palm_link");
  gripper_links.insert("l_gripper_l_finger_link");
  gripper_links.insert("l_gripper_l_finger_tip_link");
  gripper_links.insert("l_gripper_r_finger_link");
  gripper_links.insert("l_gripper_r_finger_tip_link");

  builder.Build(gripper_links, &kGripperMarkers);

  // Shift palm to origin / identity orientation.
  geometry_msgs::Pose root_pose;
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

  gripper_pub_.publish(kGripperMarkers);
}

void GraspPlanner::VisualizeGripper(const std::string& left_or_right,
                                    const geometry_msgs::Pose& pose,
                                    const std::string& frame_id) {
  // visualization_msgs::MarkerArray marker_arr = kGripperMarkers;
  // for (size_t i = 0; i < marker_arr.markers.size(); ++i) {
  //  visualization_msgs::Marker& marker = marker_arr.markers[i];
  //  marker.header.frame_id = frame_id;
  //}
}
}  // namespace pbi
