#ifndef _PBI_PROGRAM_SLICE_H_
#define _PBI_PROGRAM_SLICE_H_

#include "ros/duration.h"
#include "task_perception_msgs/ProgramSlice.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace pbi {
// A Slice represents part of a program.
// A program is simply a list of steps as demonstrated by a person. Each step
// takes some duration to execute. Each step needs to be re-timed to run
// properly on the robot, changing the duration of each step. This may affect
// the synchronization of bimanual motions.
//
// To solve this, we "slice" the program such that a slice starts when an arm
// action begins and ends when an arm action ends. Whatever the other arm is
// doing during the duration of the slice is copied over. We then retime all the
// slices individually.
//
// Example: left arm grasps object from t=0 to 10, right arm grasps object from
// t=6 to 12. Then the slices are: [0-6], [6-10], [10-12].
//
// The grasp/ungrasp steps are filled out depending on whether the slice
// starts/ends with a left/right arm action. Otherwise, they are left blank.

bool IsSliceEmpty(const task_perception_msgs::ProgramSlice& slice);

// A PlannedStep is a Step that has been planned out into a trajectory.
// Arm movements will be stored in traj.
// Gripper movements will be represented as an empty trajectory with either
// is_closing or is_opening set to true.
class PlannedStep {
 public:
  PlannedStep();
  trajectory_msgs::JointTrajectory GetTraj(const ros::Time& start_time,
                                           const ros::Time& end_time) const;
  void GetIsClosingOrOpening(const ros::Time& start_time,
                             const ros::Time& end_time, bool* is_closing,
                             bool* is_opening) const;

  trajectory_msgs::JointTrajectory traj;
  bool is_closing;
  bool is_opening;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_SLICE_H_
