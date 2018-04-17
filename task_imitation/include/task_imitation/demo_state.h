#ifndef _PBI_DEMO_STATE_H_
#define _PBI_DEMO_STATE_H_

#include <string>

#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ObjectState.h"

namespace pbi {
task_perception_msgs::ObjectState GetObjectState(
    const task_perception_msgs::DemoState& state,
    const std::string& object_name);
}  // namespace pbi

#endif  // _PBI_DEMO_STATE_H_
