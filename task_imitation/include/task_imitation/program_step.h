#ifndef _PBI_PROGRAM_STEP_H_
#define _PBI_PROGRAM_STEP_H_

#include "ros/duration.h"
#include "task_perception_msgs/Step.h"

namespace pbi {
ros::Duration GetEndTime(const task_perception_msgs::Step& step);
}  // namespace pbi

#endif  // _PBI_PROGRAM_STEP_H_
