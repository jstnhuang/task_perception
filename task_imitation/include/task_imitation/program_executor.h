#ifndef _PBI_PROGRAM_EXECUTOR_H_
#define _PBI_PROGRAM_EXECUTOR_H_

#include <map>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "pr2_actions/gripper.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "tf/transform_listener.h"

#include "task_imitation/program_slice.h"

namespace pbi {
// ProgramExecutor executes a given program on a PR2 robot.
//
// Given a program and the locations of objects in the program, it is
// responsible for planning the grasps, arm trajectories, and executing the
// motions.
class ProgramExecutor {
 public:
  ProgramExecutor();
  void Init();
  void Execute(const task_perception_msgs::Program& program,
               const std::map<std::string, task_perception_msgs::ObjectState>&
                   object_states);
  std::string planning_frame() const;

 private:
  // std::vector<Slice> ComputeSlices(
  //    const task_perception_msgs::Program& program,
  //    const std::map<std::string, task_perception_msgs::ObjectState>&
  //        object_states);

  ros::NodeHandle nh_;

  moveit::planning_interface::MoveGroup left_group_;
  moveit::planning_interface::MoveGroup right_group_;
  moveit::planning_interface::MoveGroup arms_group_;
  const std::string planning_frame_;
  pr2_actions::Gripper left_gripper_;
  pr2_actions::Gripper right_gripper_;

  ros::Publisher left_traj_pub_;
  ros::Publisher right_traj_pub_;
  ros::Publisher gripper_pub_;

  tf::TransformListener tf_listener_;
};

std::vector<Slice> SliceProgram(const task_perception_msgs::Program& program);

std::vector<geometry_msgs::Pose> SampleTrajectory(
    const std::vector<geometry_msgs::Pose>& traj);
}  // namespace pbi

#endif  // _PBI_PROGRAM_EXECUTOR_H_
