#include "task_utils/pr2_gripper_viz.h"

#include <string>

#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "robot_markers/builder.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

namespace pbi {
Pr2GripperViz::Pr2GripperViz() : markers_() {}

visualization_msgs::MarkerArray Pr2GripperViz::markers(
    const std::string& ns, const geometry_msgs::Pose& pose,
    const std::string& frame_id) const {
  Eigen::Affine3d pose_matrix;
  tf::poseMsgToEigen(pose, pose_matrix);

  visualization_msgs::MarkerArray result = markers_;
  for (size_t i = 0; i < result.markers.size(); ++i) {
    result.markers[i].ns = ns;
    result.markers[i].header.frame_id = frame_id;
    Eigen::Affine3d mesh_pose_matrix;
    tf::poseMsgToEigen(result.markers[i].pose, mesh_pose_matrix);
    tf::poseEigenToMsg(pose_matrix * mesh_pose_matrix, result.markers[i].pose);
  }
  return result;
}

void Pr2GripperViz::Init() {
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

  builder.Build(gripper_links, &markers_);

  // Shift palm to origin / identity orientation.
  geometry_msgs::Pose root_pose;
  for (size_t i = 0; i < markers_.markers.size(); ++i) {
    if (markers_.markers[i].mesh_resource.find("palm") != std::string::npos) {
      root_pose = markers_.markers[i].pose;
      break;
    }
  }
  Eigen::Affine3d gripper_pose;
  tf::poseMsgToEigen(root_pose, gripper_pose);

  for (size_t i = 0; i < markers_.markers.size(); ++i) {
    visualization_msgs::Marker& marker = markers_.markers[i];
    Eigen::Affine3d marker_pose;
    tf::poseMsgToEigen(marker.pose, marker_pose);
    Eigen::Affine3d shifted_pose = gripper_pose.inverse() * marker_pose;
    tf::poseEigenToMsg(shifted_pose, marker.pose);
  }
}
}  // namespace pbi
