#include "task_perception/grasp_planner.h"

#include <map>
#include <set>
#include <string>
#include <vector>

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "pcl/kdtree/kdtree.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "transform_graph/graph.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_perception/pcl_typedefs.h"
#include "task_perception/pr2_gripper_model.h"
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
                        const std::string& object_name,
                        TaskPerceptionContext* context,
                        geometry_msgs::Pose* pose) {
  geometry_msgs::Pose wrist_pose;
  if (left_or_right == "left") {
    wrist_pose = context->GetLeftWristPose();
  } else {
    wrist_pose = context->GetRightWristPose();
  }
  const std::string& frame_id(context->camera_info().header.frame_id);

  Pr2GripperModel gripper_model;
  gripper_model.set_pose(wrist_pose);

  visualization_msgs::MarkerArray marker_arr;
  VisualizeGripper(left_or_right, wrist_pose, frame_id, &marker_arr);
  gripper_model.ToMarkerArray(frame_id, &marker_arr);
  gripper_pub_.publish(marker_arr);

  KdTreeP::Ptr object_tree = context->GetObjectTree(object_name);
  PointP wrist_point;
  wrist_point.x = wrist_pose.position.x;
  wrist_point.y = wrist_pose.position.y;
  wrist_point.z = wrist_pose.position.z;
  std::vector<int> indices;
  std::vector<float> sq_distances;
  object_tree->nearestKSearch(wrist_point, 1, indices, sq_distances);
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
}

void GraspPlanner::VisualizeGripper(
    const std::string& left_or_right, const geometry_msgs::Pose& pose,
    const std::string& frame_id, visualization_msgs::MarkerArray* marker_arr) {
  Eigen::Affine3d pose_transform;
  tf::poseMsgToEigen(pose, pose_transform);

  for (size_t i = 0; i < kGripperMarkers.markers.size(); ++i) {
    visualization_msgs::Marker marker = kGripperMarkers.markers[i];
    marker.header.frame_id = frame_id;
    marker.ns = left_or_right;

    Eigen::Affine3d marker_pose;
    tf::poseMsgToEigen(marker.pose, marker_pose);
    Eigen::Affine3d shifted_pose = pose_transform * marker_pose;
    tf::poseEigenToMsg(shifted_pose, marker.pose);
    marker_arr->markers.push_back(marker);
  }
}
}  // namespace pbi
