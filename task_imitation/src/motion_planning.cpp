#include "task_imitation/motion_planning.h"

#include <sstream>

#include "moveit_msgs/MoveItErrorCodes.h"
#include "rapid_manipulation/moveit_error_code.h"
#include "rapid_ros/params.h"
#include "ros/ros.h"

#include "task_imitation/ik.h"

using geometry_msgs::Pose;
using moveit::planning_interface::MoveGroup;
using moveit::planning_interface::MoveItErrorCode;
using trajectory_msgs::JointTrajectoryPoint;

namespace pbi {
std::string PlanToPose(MoveGroup& group,
                       const robot_state::RobotState& start_state,
                       const geometry_msgs::Pose& gripper_pose, int num_tries,
                       MoveGroup::Plan* plan) {
  double max_planning_time =
      rapid::GetDoubleParamOrThrow("task_imitation/max_planning_time");
  int num_attempts =
      rapid::GetDoubleParamOrThrow("task_imitation/num_planning_attempts");
  group.setStartState(start_state);
  group.setPoseTarget(gripper_pose);
  group.setPlanningTime(max_planning_time);
  group.setNumPlanningAttempts(num_attempts);

  MoveItErrorCode error;
  for (int i = 0; i < num_tries; ++i) {
    error = group.plan(*plan);
    if (rapid::IsSuccess(error)) {
      return "";
    } else if (i < num_tries - 1) {
      ROS_WARN("Planning attempt %d of %d failed", i + 1, num_tries);
    } else {
      ROS_ERROR("Planning attempt %d of %d failed", i + 1, num_tries);
    }
  }
  return rapid::ErrorString(error);
}

std::string PlanCartesianToPose(moveit::planning_interface::MoveGroup& group,
                                const robot_state::RobotState& start_state,
                                const geometry_msgs::Pose& gripper_pose,
                                int num_tries,
                                moveit_msgs::RobotTrajectory* plan) {
  std::vector<Pose> gripper_poses(1);
  gripper_poses[0] = gripper_pose;
  return PlanCartesianToPoses(group, start_state, gripper_poses, num_tries,
                              plan);
}

std::string PlanCartesianToPoses(
    moveit::planning_interface::MoveGroup& group,
    const robot_state::RobotState& start_state,
    const std::vector<geometry_msgs::Pose>& gripper_poses, int num_tries,
    moveit_msgs::RobotTrajectory* plan) {
  const double eef_threshold =
      rapid::GetDoubleParamOrThrow("task_imitation/cart_path_eef_threshold");
  const double jump_threshold =
      rapid::GetDoubleParamOrThrow("task_imitation/cart_path_jump_threshold");
  const bool kAvoidCollisions = true;
  group.setStartState(start_state);

  std::string error("");
  for (int attempt = 0; attempt < num_tries; attempt++) {
    moveit_msgs::MoveItErrorCodes error_code;
    double fraction =
        group.computeCartesianPath(gripper_poses, eef_threshold, 0.0, *plan,
                                   kAvoidCollisions, &error_code);
    if (!rapid::IsSuccess(error_code)) {
      error = rapid::ErrorString(error_code);
      ROS_WARN("Planning attempt %d of %d failed: %s", attempt, num_tries,
               error.c_str());
      continue;
    } else if (gripper_poses.size() > 0 && fraction < 0.95) {
      std::stringstream ss;
      ss << "Planned " << fraction * 100 << "% of arm trajectory";
      error = ss.str();
      ROS_WARN("Planning attempt %d of %d failed: %s", attempt, num_tries,
               error.c_str());
      int ik_count = 0;
      for (size_t i = 0; i < gripper_poses.size(); ++i) {
        if (HasIk(group, gripper_poses[i])) {
          ik_count++;
        }
      }
      ROS_INFO("%d of %zu poses have IK (%f%%)", ik_count, gripper_poses.size(),
               ik_count * 100.0 / gripper_poses.size());
      continue;
    } else {
      // Check the amount of change in configuration space
      double sq_max_jump = 0;
      for (size_t i = 0; i + 1 < plan->joint_trajectory.points.size(); i++) {
        const JointTrajectoryPoint current_pt =
            plan->joint_trajectory.points[i];
        const JointTrajectoryPoint next_pt =
            plan->joint_trajectory.points[i + 1];
        double sq_config_dist = 0;
        for (size_t j = 0; j + 1 < current_pt.positions.size(); j++) {
          double dist = current_pt.positions[j] - next_pt.positions[j];
          sq_config_dist += dist * dist;
        }
        if (sq_config_dist > sq_max_jump) {
          sq_max_jump = sq_config_dist;
        }
      }
      double max_jump = sqrt(sq_max_jump);
      if (max_jump > jump_threshold) {
        ROS_WARN("Trajectory includes a C-space jump of %f! Be careful!",
                 max_jump);
      } else {
        ROS_INFO("Max C-space jump in trajectory: %f", max_jump);
      }
      ROS_INFO("Planned %f%% of arm trajectory (%zu -> %zu pts)",
               fraction * 100, gripper_poses.size(),
               plan->joint_trajectory.points.size());
      return "";
    }
  }
  return error;
}
}  // namespace pbi
