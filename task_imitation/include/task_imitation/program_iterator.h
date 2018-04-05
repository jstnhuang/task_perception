#ifndef _PBI_PROGRAM_ITERATOR_H_
#define _PBI_PROGRAM_ITERATOR_H_

#include <vector>

#include "boost/optional.hpp"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include "task_imitation/program_slice.h"

namespace pbi {
// Iterates through events in a list of steps.
class ProgramIterator {
 public:
  explicit ProgramIterator(const std::vector<PlannedStep>& steps);

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
  ros::Time time();

  // Gets the current step the iterator is pointing to.
  //
  // Do not call if IsDone() is true.
  PlannedStep step();

  // Gets the trajectory point the iterator is pointing to, if any.
  //
  // Do not call if IsDone() is true.
  boost::optional<
      std::pair<trajectory_msgs::JointTrajectoryPoint, ros::Duration> >
  trajectory_point();

 private:
  const std::vector<PlannedStep>& steps_;
  size_t step_i_;
  size_t traj_i_;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_ITERATOR_H_
