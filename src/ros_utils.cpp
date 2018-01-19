#include "task_perception/ros_utils.h"

#include <string>

#include "ros/ros.h"

namespace pbi {
bool GetParam(const std::string& name, float* val) {
  bool success = ros::param::get(name, *val);
  if (!success) {
    LogGetParamFail(name);
  }
  return success;
}

bool GetParam(const std::string& name, bool* val) {
  bool success = ros::param::get(name, *val);
  if (!success) {
    LogGetParamFail(name);
  }
  return success;
}

void LogGetParamFail(const std::string& name) {
  ROS_ERROR("Failed to get param \"%s\"", name.c_str());
}
}  // namespace pbi
