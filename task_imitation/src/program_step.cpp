#include "task_imitation/program_step.h"

#include "ros/ros.h"

#include "task_imitation/program_constants.h"

namespace msgs = task_perception_msgs;

namespace pbi {
ros::Duration GetEndTime(const task_perception_msgs::Step& step) {
  if (step.type == msgs::Step::GRASP) {
    return step.start_time;
  } else if (step.type == msgs::Step::UNGRASP) {
    return step.start_time + ros::Duration(kUngraspDuration);
  } else if (step.type == msgs::Step::MOVE_TO_POSE) {
    if (step.times_from_start.size() > 0) {
      return step.start_time + step.times_from_start.back();
    } else {
      ROS_ASSERT(false);
      return step.start_time;
    }
  } else if (step.type == msgs::Step::FOLLOW_TRAJECTORY) {
    if (step.times_from_start.size() > 0) {
      return step.start_time + step.times_from_start.back();
    } else {
      ROS_ASSERT(false);
      return step.start_time;
    }
  }
  ROS_ASSERT_MSG(false, "Unknown step type %s", step.type.c_str());
  return ros::Duration(0);
}
}  // namespace pbi
