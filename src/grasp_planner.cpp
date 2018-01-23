#include "task_perception/grasp_planner.h"

#include <map>
#include <set>
#include <string>

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
  joint_positions["r_gripper_joint"] = 0.087;
  builder.SetJointPositions(joint_positions);

  std::set<std::string> gripper_links;
  gripper_links.insert("r_gripper_palm_link");
  gripper_links.insert("r_gripper_l_finger_link");
  gripper_links.insert("r_gripper_l_finger_tip_link");
  gripper_links.insert("r_gripper_r_finger_link");
  gripper_links.insert("r_gripper_r_finger_tip_link");

  builder.Build(gripper_links, &kGripperMarkers);
}
}  // namespace pbi
