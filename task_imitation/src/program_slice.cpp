#include "task_imitation/program_slice.h"

#include "task_perception_msgs/ProgramSlice.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

using trajectory_msgs::JointTrajectory;
using trajectory_msgs::JointTrajectoryPoint;

namespace pbi {
bool IsSliceEmpty(const task_perception_msgs::ProgramSlice& slice) {
  return slice.left_traj.points.empty() && slice.right_traj.points.empty();
}

PlannedStep::PlannedStep() : traj(), is_closing(false), is_opening(false) {}

JointTrajectory PlannedStep::GetTraj(const ros::Time& start_time,
                                     const ros::Time& end_time) {
  // A common case to reject early is when end_time is exactly equal to the
  // start time of this trajectory. This is because we are always trying to
  // extract trajectories from the next step.
  if (end_time <= traj.header.stamp) {
    JointTrajectory blank;
    return blank;
  }

  JointTrajectory result;
  result.header.stamp = start_time;
  ros::Time pt_start = traj.header.stamp;
  ros::Time pt_end(0);
  for (size_t i = 0; i < traj.points.size(); ++i) {
    JointTrajectoryPoint pt = traj.points[i];
    pt_end = traj.header.stamp + pt.time_from_start;

    // Check if the request overlaps with this trajectory point
    if (start_time <= pt_end && end_time >= pt_start) {
      ros::Time result_pt_start = pt_start;
      ros::Time result_pt_end = pt_end;
      if (end_time < pt_end) {
        result_pt_end = end_time;
      }
      if (start_time > pt_start) {
        result_pt_start = start_time;
      }
      pt.time_from_start = result_pt_end - result_pt_start;
      if (!pt.time_from_start.isZero()) {
        result.points.push_back(pt);
      }
    }
    if (pt_start > end_time) {
      break;
    }

    pt_start = pt_end;
  }
  return result;
}
}  // namespace pbi
