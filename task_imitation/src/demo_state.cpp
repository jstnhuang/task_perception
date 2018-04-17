#include "task_imitation/demo_state.h"

#include "ros/ros.h"

namespace msgs = task_perception_msgs;

namespace pbi {
msgs::ObjectState GetObjectState(const msgs::DemoState& state,
                                 const std::string& object_name) {
  for (size_t i = 0; i < state.object_states.size(); ++i) {
    const msgs::ObjectState object = state.object_states[i];
    if (object.name == object_name) {
      return object;
    }
  }
  ROS_ERROR("No object \"%s\" in state!", object_name.c_str());
  msgs::ObjectState kBlank;
  return kBlank;
}

}  // namespace pbi
