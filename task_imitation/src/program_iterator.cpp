#include "task_imitation/program_iterator.h"

#include <utility>
#include <vector>

#include "boost/optional.hpp"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include "task_imitation/program_slice.h"

using boost::optional;
using trajectory_msgs::JointTrajectoryPoint;

namespace pbi {
ProgramIterator::ProgramIterator(const std::vector<PlannedStep>& steps)
    : steps_(steps), step_i_(0), traj_i_(0) {}

void ProgramIterator::Begin() {
  step_i_ = 0;
  traj_i_ = 0;
}

void ProgramIterator::Advance() {
  if (IsDone()) {
    ROS_WARN("Called Advance() on finished ProgramIterator");
    return;
  }
  const PlannedStep& current = step();

  ROS_ASSERT(current.traj.points.size() > 0);
  if (traj_i_ < current.traj.points.size() - 1) {
    ++traj_i_;
  } else {
    traj_i_ = 0;
    ++step_i_;
  }
}

bool ProgramIterator::IsDone() { return step_i_ >= steps_.size(); }

ros::Time ProgramIterator::time() {
  ROS_ASSERT(!IsDone());

  const PlannedStep& current = step();
  const ros::Time start = current.traj.header.stamp;
  const JointTrajectoryPoint pt = current.traj.points[traj_i_];
  return start + pt.time_from_start;
}

PlannedStep ProgramIterator::step() {
  ROS_ASSERT(!IsDone());
  return steps_[step_i_];
}

optional<std::pair<JointTrajectoryPoint, ros::Duration> >
ProgramIterator::trajectory_point() {
  ROS_ASSERT(!IsDone());
  /*
  const PlannedStep& current = step();
  if (current.type != msgs::Step::FOLLOW_TRAJECTORY) {
    return boost::none;
  }
  return std::make_pair<Pose, ros::Duration>(current.ee_trajectory[traj_i_],
                                             current.times_from_start[traj_i_]);
                                             */
  return boost::none;
}
}  // namespace pbi
