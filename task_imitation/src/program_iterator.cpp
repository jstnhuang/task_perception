#include "task_imitation/program_iterator.h"

#include <utility>
#include <vector>

#include "boost/optional.hpp"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "task_perception_msgs/Step.h"

namespace msgs = task_perception_msgs;
using boost::optional;
using geometry_msgs::Pose;

namespace pbi {
ProgramIterator::ProgramIterator(const std::vector<msgs::Step>& steps)
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
  const msgs::Step& current = step();
  if (current.type == msgs::Step::GRASP) {
    ++step_i_;
  } else if (current.type == msgs::Step::FOLLOW_TRAJECTORY) {
    if (traj_i_ < current.object_trajectory.size() - 1) {
      ++traj_i_;
    } else {
      traj_i_ = 0;
      ++step_i_;
    }
  } else if (current.type == msgs::Step::UNGRASP) {
    ++step_i_;
  }
}

bool ProgramIterator::IsDone() { return step_i_ >= steps_.size(); }

ros::Duration ProgramIterator::time() {
  ROS_ASSERT(!IsDone());

  const msgs::Step& current = step();
  if (current.type == msgs::Step::GRASP) {
    return current.start_time;
  } else if (current.type == msgs::Step::FOLLOW_TRAJECTORY) {
    return current.start_time + current.times_from_start[traj_i_];
  } else if (current.type == msgs::Step::UNGRASP) {
    return current.start_time;
  } else {
    ROS_ASSERT_MSG(false, "Unsupported action type \"%s\"",
                   current.type.c_str());
    ros::Duration zero;
    return zero;
  }
}

msgs::Step ProgramIterator::step() {
  ROS_ASSERT(!IsDone());
  return steps_[step_i_];
}

optional<std::pair<Pose, ros::Duration> > ProgramIterator::trajectory_point() {
  ROS_ASSERT(!IsDone());
  const msgs::Step& current = step();
  if (current.type != msgs::Step::FOLLOW_TRAJECTORY) {
    return boost::none;
  }
  return std::make_pair<Pose, ros::Duration>(current.object_trajectory[traj_i_],
                                             current.times_from_start[traj_i_]);
}
}  // namespace pbi
