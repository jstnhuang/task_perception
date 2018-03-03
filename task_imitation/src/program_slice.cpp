#include "task_imitation/program_slice.h"

#include "task_perception_msgs/Step.h"

namespace msgs = task_perception_msgs;

namespace pbi {
Slice::Slice() : grasp(), left_traj(), right_traj(), ungrasp() {}

void Slice::Reset() {
  msgs::Step blank;
  grasp = blank;
  left_traj = blank;
  right_traj = blank;
  ungrasp = blank;
}

void Slice::FixTrajectories() {
  if (!left_traj.start_time.isZero() &&
      left_traj.object_trajectory.size() > 0) {
    ros::Duration offset = left_traj.times_from_start[0];
    left_traj.start_time += offset;
    for (size_t i = 0; i < left_traj.times_from_start.size(); ++i) {
      left_traj.times_from_start[i] -= (offset - ros::Duration(0.033));
    }
  }
  if (!right_traj.start_time.isZero() &&
      right_traj.object_trajectory.size() > 0) {
    ros::Duration offset = right_traj.times_from_start[0];
    right_traj.start_time += offset;
    for (size_t i = 0; i < right_traj.times_from_start.size(); ++i) {
      right_traj.times_from_start[i] -= (offset - ros::Duration(0.033));
    }
  }
}
}  // namespace pbi
