#ifndef _PBI_PROGRAM_EXECUTOR_H_
#define _PBI_PROGRAM_EXECUTOR_H_

#include <map>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "rapid_pr2/gripper.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/ProgramSlice.h"
#include "tf/transform_listener.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "task_imitation/grasp_planner.h"
#include "task_imitation/program_slice.h"

namespace pbi {
// ProgramExecutor executes a given program on a PR2 robot.
//
// Given a program and the locations of objects in the program, it is
// responsible for planning the grasps, arm trajectories, and executing the
// motions.
class ProgramExecutor {
 public:
  ProgramExecutor(moveit::planning_interface::MoveGroup& left_group,
                  moveit::planning_interface::MoveGroup& right_group);
  void Init();
  void Execute(const task_perception_msgs::Program& program,
               const std::map<std::string, task_perception_msgs::ObjectState>&
                   object_states);
  std::string planning_frame() const;

 private:
  std::vector<task_perception_msgs::ProgramSlice> RetimeSlices(
      const std::vector<task_perception_msgs::ProgramSlice>& slices);

  ros::NodeHandle nh_;

  moveit::planning_interface::MoveGroup& left_group_;
  moveit::planning_interface::MoveGroup& right_group_;
  moveit::planning_interface::MoveGroup arms_group_;
  const std::string planning_frame_;
  rapid::pr2::Gripper left_gripper_;
  rapid::pr2::Gripper right_gripper_;

  ros::Publisher left_traj_pub_;
  ros::Publisher right_traj_pub_;
  ros::Publisher gripper_pub_;
  ros::Publisher slice_pub_;

  tf::TransformListener tf_listener_;
};

std::vector<task_perception_msgs::ProgramSlice> SliceProgram(
    const std::vector<PlannedStep>& left_steps,
    const std::vector<PlannedStep>& right_steps);

// Given an object-relative grasp and an object trajectory, computes the
// trajectory of the grasp.
//
// ee_trajectory: The trajectory of the end-effector relative to the initial
//  object pose.
// current_obj_pose: The pose of the object at execution time, in the planning
//  frame.
std::vector<geometry_msgs::Pose> ComputeGraspTrajectory(
    const std::vector<geometry_msgs::Pose>& ee_trajectory,
    const geometry_msgs::Pose& current_obj_pose);

std::vector<geometry_msgs::Pose> SampleTrajectory(
    const std::vector<geometry_msgs::Pose>& traj);

ros::Duration ComputeTrajectoryTime(
    const trajectory_msgs::JointTrajectory& traj);

// Plan trajectories for each step
// GRASP steps will have added steps to move to a pre-grasp and grasp pose.
// UNGRASP steps will have an added step to move to a post-grasp pose.
// MOVE_TO_POSE steps will be scaled to match the demonstration time.
std::vector<PlannedStep> PlanSteps(
    const std::vector<task_perception_msgs::Step>& steps,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group);

// step, object_states, group, and start_time are input parameters.
// start_state is both an input and an output. As as input, it specifies the
// start state before the step. As an output, it specifies what the updated
// state is after the step.
std::vector<PlannedStep> PlanGraspStep(
    const task_perception_msgs::Step& step,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group, const ros::Time& start_time,
    robot_state::RobotStatePtr start_state);

std::vector<PlannedStep> PlanUngraspStep(
    const task_perception_msgs::Step& step,
    moveit::planning_interface::MoveGroup& group, const ros::Time& start_time,
    robot_state::RobotStatePtr start_state);

PlannedStep PlanFollowTrajectoryStep(
    const task_perception_msgs::Step& step,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group, const ros::Time& start_time,
    robot_state::RobotStatePtr start_state);

PlannedStep PlanMoveToPoseStep(
    const task_perception_msgs::Step& step,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group, const ros::Time& start_time,
    robot_state::RobotStatePtr start_state);

// Checks the validity of a trajectory message.
// Currently, only checks that the time_from_starts are monotonically increasing
bool IsValidTrajectory(const trajectory_msgs::JointTrajectory& traj);
}  // namespace pbi

#endif  // _PBI_PROGRAM_EXECUTOR_H_
