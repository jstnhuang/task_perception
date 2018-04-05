#ifndef _PBI_PROGRAM_SLICE_H_
#define _PBI_PROGRAM_SLICE_H_

#include "ros/duration.h"
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
class Slice {
 public:
  Slice();

  // Resets this slice so that all the steps are blank messages.
  void Reset();

  // Shift left and right trajectories such that time_from_start always starts
  // at 0.03. Also adjusts the start_time to match.
  //
  // E.g., if we slice with a trajectory from t=[1, 10] into t=[1, 6] and
  // t=[6,10], then the second trajectory will have a start_time of 1 and its
  // first time_from_start will be 5. After FixTrajectories(), it will have a
  // start time of 6 and a time_from_start of 0.03.
  //
  // TODO: Hard-coded 0.03s start time assumes trajectory data is recorded at
  // 30Hz. We lose this information in the slicing. To regain this information,
  // we need the previous slice.
  // void FixTrajectories();

  bool IsEmpty() const;

  trajectory_msgs::JointTrajectory left_traj;
  trajectory_msgs::JointTrajectory right_traj;
  bool is_left_closing;
  bool is_left_opening;
  bool is_right_closing;
  bool is_right_opening;
};

// A PlannedStep is a Step that has been planned out into a trajectory.
// Arm movements will be stored in traj.
// Gripper movements will be represented as an empty trajectory with either
// is_closing or is_opening set to true.
class PlannedStep {
 public:
  PlannedStep();
  trajectory_msgs::JointTrajectory GetTraj(const ros::Time& start_time,
                                           const ros::Time& end_time);

  trajectory_msgs::JointTrajectory traj;
  bool is_closing;
  bool is_opening;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_SLICE_H_
