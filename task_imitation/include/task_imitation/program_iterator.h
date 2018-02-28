#ifndef _PBI_PROGRAM_ITERATOR_H_
#define _PBI_PROGRAM_ITERATOR_H_

#include <vector>

#include "boost/optional.hpp"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "task_perception_msgs/Step.h"

namespace pbi {
// Iterates through events in a list of steps. An event is either a grasp, an
// ungrasp, or a trajectory point.
class ProgramIterator {
 public:
  explicit ProgramIterator(
      const std::vector<task_perception_msgs::Step>& steps);

  // This must be called to initialize the iterator.
  void Begin();

  // Calling this will advance the iterator to point to the next event.
  void Advance();

  // Returns true if the iterator has advanced past the last event.
  bool IsDone();

  // Gets the time of the current event. For grasp/ungrasp events, this is the
  // start_time. For trajectory points, this is the start_time plus the
  // time_from_start of the point.
  //
  // Do not call if IsDone() is true.
  ros::Duration time();

  // Gets the current step the iterator is pointing to.
  //
  // Do not call if IsDone() is true.
  task_perception_msgs::Step step();

  // Gets the trajectory point the iterator is pointing to, if any.
  //
  // Do not call if IsDone() is true.
  boost::optional<std::pair<geometry_msgs::Pose, ros::Duration> >
  trajectory_point();

 private:
  const std::vector<task_perception_msgs::Step>& steps_;
  size_t step_i_;
  size_t traj_i_;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_ITERATOR_H_
