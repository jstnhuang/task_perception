#ifndef _PBI_PROGRAM_PLANNER_H_
#define _PBI_PROGRAM_PLANNER_H_

#include "moveit/move_group_interface/move_group.h"
#include "task_perception_msgs/Program.h"
#include "tf/transform_listener.h"

namespace pbi {
// Converts a msgs::Program into an executable plan.
//
// Steps:
// - Split the arm motions into spans such that no arm starts moving while the
//   other is in motion.
// - Run dual-arm planning for each span
// - The trajectories will be re-timed, so time-shift the later trajectories
class ProgramPlanner {
 public:
  ProgramPlanner();
  void Process(const task_perception_msgs::Program& program);

 private:
  moveit::planning_interface::MoveGroup move_group_;
  tf::TransformListener tf_listener_;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_PLANNER_H_
