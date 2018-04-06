#ifndef _PBI_BIMANUAL_MANIPULATION_H_
#define _PBI_BIMANUAL_MANIPULATION_H_

#include "moveit/move_group_interface/move_group.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace pbi {
// Fill velocities, accelerations, and effort with 0s if they are not specified.
// Assumes that positions is filled out.
trajectory_msgs::JointTrajectoryPoint StandardizePoint(
    const trajectory_msgs::JointTrajectoryPoint& pt);

// Return a trajectory consisting of just the start state.
// This trajectory can be merged in MergeTrajectories to move just one arm.
trajectory_msgs::JointTrajectory GetNonMovingTrajectory(
    moveit::planning_interface::MoveGroup& group,
    const ros::Duration& duration);

// Merge trajectories for left and right arm into one trajectory that can be
// executed by the arms group.
//
// Precondition: each trajectory must have at least one point.
// Precondition: each trajectory starts at the same time.
//
// If you only want to move one arm, call GetNonMovingTrajectory to get a
// trajectory that holds the other arm still (or just use the MoveGroup client
// for the moving arm).
trajectory_msgs::JointTrajectory MergeTrajectories(
    const trajectory_msgs::JointTrajectory& left_traj,
    const trajectory_msgs::JointTrajectory& right_traj);
}  // namespace pbi

#endif  // _PBI_BIMANUAL_MANIPULATION_H_
