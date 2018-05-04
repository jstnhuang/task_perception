#include "task_imitation/program_slice.h"

#include <stdexcept>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

using trajectory_msgs::JointTrajectory;
using trajectory_msgs::JointTrajectoryPoint;

namespace pbi {
bool IsSliceEmpty(const task_perception_msgs::ProgramSlice& slice) {
  return slice.left_traj.points.empty() && slice.right_traj.points.empty();
}

PlannedStep::PlannedStep() : traj(), is_closing(false), is_opening(false) {}

JointTrajectory PlannedStep::GetTraj(const ros::Time& start_time,
                                     const ros::Time& end_time) const {
  // A common case to reject early is when end_time is exactly equal to the
  // start time of this trajectory. This is because we are always trying to
  // extract trajectories from the next step.
  if (end_time <= traj.header.stamp) {
    JointTrajectory blank;
    return blank;
  }

  JointTrajectory result;
  result.header.stamp = start_time;
  result.joint_names = traj.joint_names;
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
      pt.time_from_start = result_pt_end - start_time;
      result.points.push_back(pt);
    }
    if (pt_start > end_time) {
      break;
    }

    pt_start = pt_end;
  }
  return result;
}

void PlannedStep::GetIsClosingOrOpening(const ros::Time& start_time,
                                        const ros::Time& end_time,
                                        bool* is_closing_result,
                                        bool* is_opening_result) const {
  ROS_ASSERT(start_time <= end_time);
  ROS_ASSERT(traj.points.size() > 0);
  ros::Time start = traj.header.stamp;
  ros::Time end = start + traj.points.back().time_from_start;

  // The requested interval must either be entirely outside this step, or
  // entirely inside.
  bool bad_condition1 = start_time < start && end_time > start;
  bool bad_condition2 =
      start_time >= start && start_time < end && end_time > end;
  if (bad_condition1 || bad_condition2) {
    ROS_ERROR("step: [%f, %f], requested: [%f, %f]", start.toSec(), end.toSec(),
              start_time.toSec(), end_time.toSec());
    throw std::invalid_argument(
        "Requested interval must be entirely outside or entirely inside the "
        "PlannedStep");
  }

  if (start_time >= start && end_time <= end) {
    *is_closing_result = is_closing;
    *is_opening_result = is_opening;
  } else {
    *is_closing_result = false;
    *is_opening_result = false;
  }
}
}  // namespace pbi
