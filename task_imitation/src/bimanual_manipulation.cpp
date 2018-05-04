#include "task_imitation/bimanual_manipulation.h"

#include <limits.h>
#include <string>
#include <vector>

#include "moveit/robot_state/robot_state.h"

using moveit_msgs::RobotTrajectory;
using trajectory_msgs::JointTrajectory;
using trajectory_msgs::JointTrajectoryPoint;

namespace pbi {
namespace {
int clamp(int i, int min, int max) {
  if (i < min) {
    return min;
  }
  if (i > max) {
    return max;
  }
  return i;
}
}  // namespace

JointTrajectoryPoint StandardizePoint(const JointTrajectoryPoint& pt) {
  JointTrajectoryPoint result = pt;
  int dims = result.positions.size();
  if (result.velocities.size() == 0) {
    result.velocities.resize(dims, 0);
  }
  if (result.accelerations.size() == 0) {
    result.accelerations.resize(dims, 0);
  }
  if (result.effort.size() == 0) {
    result.effort.resize(dims, 0);
  }
  return result;
}

trajectory_msgs::JointTrajectory GetNonMovingTrajectory(
    moveit::planning_interface::MoveGroup& group,
    const ros::Duration& duration) {
  trajectory_msgs::JointTrajectory result;
  robot_state::RobotStatePtr current = group.getCurrentState();
  result.joint_names = group.getJointNames();
  JointTrajectoryPoint pt;
  pt.time_from_start = duration;
  for (size_t i = 0; i < result.joint_names.size(); ++i) {
    const std::string& joint_name = result.joint_names[i];
    pt.positions.push_back(*current->getJointPositions(joint_name));
    pt.velocities.push_back(0);
    pt.accelerations.push_back(0);
    pt.effort.push_back(0);
  }
  result.points.push_back(pt);
  return result;
}

JointTrajectory MergeTrajectories(const JointTrajectory& left_traj,
                                  const JointTrajectory& right_traj) {
  JointTrajectory result;
  if (left_traj.points.size() == 0) {
    ROS_ERROR("Left trajectory must have at least one point");
    return result;
  } else if (right_traj.points.size() == 0) {
    ROS_ERROR("Right trajectory must have at least one point");
    return result;
  }

  result.joint_names = left_traj.joint_names;
  result.joint_names.insert(result.joint_names.end(),
                            right_traj.joint_names.begin(),
                            right_traj.joint_names.end());

  // Walk through both trajectories in order of time.
  size_t left_i = 0;
  size_t right_i = 0;
  const std::vector<JointTrajectoryPoint>& left_pts = left_traj.points;
  const std::vector<JointTrajectoryPoint>& right_pts = right_traj.points;
  while (left_i < left_pts.size() || right_i < right_pts.size()) {
    JointTrajectoryPoint left_pt;
    double left_time = std::numeric_limits<double>::max();
    if (left_i < left_pts.size()) {
      left_pt = StandardizePoint(left_pts[left_i]);
      left_time = left_pt.time_from_start.toSec();
    } else if (left_i == left_pts.size()) {
      left_pt = StandardizePoint(left_pts[left_i - 1]);
    }

    JointTrajectoryPoint right_pt;
    double right_time = std::numeric_limits<double>::max();
    if (right_i < right_pts.size()) {
      right_pt = StandardizePoint(right_pts[right_i]);
      right_time = right_pt.time_from_start.toSec();
    } else if (right_i == right_pts.size()) {
      right_pt = StandardizePoint(right_pts[right_i - 1]);
    }

    // If the left point is sooner, then merge the left point and the previous
    // right point, and vice versa. If the points have identical start times,
    // then merge them into a single point.
    JointTrajectoryPoint pt;
    if (left_time <= right_time) {
      pt.time_from_start = ros::Duration(left_time);
    } else if (right_time < left_time) {
      pt.time_from_start = ros::Duration(right_time);
    }

    pt.positions = left_pt.positions;
    pt.positions.insert(pt.positions.end(), right_pt.positions.begin(),
                        right_pt.positions.end());
    pt.velocities = left_pt.velocities;
    pt.velocities.insert(pt.velocities.end(), right_pt.velocities.begin(),
                         right_pt.velocities.end());
    pt.accelerations = left_pt.accelerations;
    pt.accelerations.insert(pt.accelerations.end(),
                            right_pt.accelerations.begin(),
                            right_pt.accelerations.end());
    pt.effort = left_pt.effort;
    pt.effort.insert(pt.effort.end(), right_pt.effort.begin(),
                     right_pt.effort.end());
    result.points.push_back(pt);

    if (left_time < right_time) {
      ++left_i;
    } else if (right_time < left_time) {
      ++right_i;
    } else {
      ++left_i;
      ++right_i;
    }
  }
  return result;
}
}  // namespace pbi
