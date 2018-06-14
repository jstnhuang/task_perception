#ifndef _PBI_MOTION_PLANNING_H_
#define _PBI_MOTION_PLANNING_H_

#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/RobotTrajectory.h"

namespace pbi {
// Plans a trajectory to a pose using MoveIt.
// group: The MoveGroup instance to use
// start_state: The start state of the robot for this pose
// gripper_pose: The pose of the end-effector to plan for
// num_tries: The maximum number of times to try planning
// plan: The output plan, if successful.
//
// Returns "" if a plan was found, error string otherwise.
std::string PlanToPose(moveit::planning_interface::MoveGroup& group,
                       const robot_state::RobotState& start_state,
                       const geometry_msgs::Pose& gripper_pose, int num_tries,
                       moveit::planning_interface::MoveGroup::Plan* plan);

// Plans a straight-line path to a pose using MoveIt.
std::string PlanCartesianToPose(moveit::planning_interface::MoveGroup& group,
                                const robot_state::RobotState& start_state,
                                const geometry_msgs::Pose& gripper_pose,
                                int num_tries,
                                moveit_msgs::RobotTrajectory* plan);

std::string PlanCartesianToPoses(
    moveit::planning_interface::MoveGroup& group,
    const robot_state::RobotState& start_state,
    const std::vector<geometry_msgs::Pose>& gripper_poses, int num_tries,
    moveit_msgs::RobotTrajectory* plan);
}  // namespace pbi

#endif  // _PBI_MOTION_PLANNING_H_
