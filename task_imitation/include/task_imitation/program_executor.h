#ifndef _PBI_PROGRAM_EXECUTOR_H_
#define _PBI_PROGRAM_EXECUTOR_H_

#include <map>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "rapid_pr2/gripper.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/ProgramSlice.h"
#include "trajectory_msgs/JointTrajectory.h"

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
  // Returns error message or "" on success.
  std::string Execute(
      const task_perception_msgs::Program& program,
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
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      left_arm_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      right_arm_;

  ros::Publisher slice_pub_;
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
    moveit::planning_interface::MoveGroup& group, std::string* error_out);

// step, object_states, group, and start_time are input parameters.
// start_state is both an input and an output. As as input, it specifies the
// start state before the step. As an output, it specifies what the updated
// state is after the step.
//
// Timing: the program generator allocates 2 seconds for the grasp to happen,
// starting from start_time + step.header.stamp. However, no time is allocated
// for the pre-grasp and move-to-grasp motions. prev_end specifies when the
// previous step ends, so the pre-grasp and move-to-grasp must happen between
// prev_end and start_time + step.header.stamp.
std::vector<PlannedStep> PlanGraspStep(
    const task_perception_msgs::Step& step,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group, const ros::Time& plan_start,
    const ros::Time& prev_end, robot_state::RobotStatePtr start_state,
    std::string* error_out);

// next_start_time gives the start time of the next step. The program generator
// allocates 2 seconds for the ungrasp step, but no time for the post-grasp
// movement. The post-grasp must be scaled to fit in between the end of the
// ungrasp and the start of the next step.
std::vector<PlannedStep> PlanUngraspStep(
    const task_perception_msgs::Step& step,
    moveit::planning_interface::MoveGroup& group, const ros::Time& plan_start,
    const ros::Time& next_start, robot_state::RobotStatePtr start_state,
    std::string* error_out);

PlannedStep PlanFollowTrajectoryStep(
    const task_perception_msgs::Step& step,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group, const ros::Time& plan_start,
    const ros::Time& next_start, robot_state::RobotStatePtr start_state,
    std::string* error_out);

PlannedStep PlanMoveToPoseStep(
    const task_perception_msgs::Step& step,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group, const ros::Time& plan_start,
    const ros::Time& next_start, robot_state::RobotStatePtr start_state,
    std::string* error_out);

// Checks the validity of a trajectory message.
// Currently, only checks that the time_from_starts are monotonically increasing
bool IsValidTrajectory(const trajectory_msgs::JointTrajectory& traj);

// Returns error string or "" if valid.
std::string ValidatePlannedSteps(const std::vector<PlannedStep>& planned_steps);

void PrintPlan(const std::vector<PlannedStep>& left_steps,
               const std::vector<PlannedStep>& right_steps);
ros::Time GetStartOfSlices(
    const std::vector<task_perception_msgs::ProgramSlice>& slices);
void PrintSlices(const std::vector<task_perception_msgs::ProgramSlice>& slices);

trajectory_msgs::JointTrajectory RetimeTrajectory(
    const trajectory_msgs::JointTrajectory& traj,
    moveit::planning_interface::MoveGroup& group,
    moveit::core::RobotStatePtr start_state);

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
}  // namespace pbi

#endif  // _PBI_PROGRAM_EXECUTOR_H_
