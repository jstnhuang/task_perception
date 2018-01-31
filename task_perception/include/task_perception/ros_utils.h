#ifndef _PBI_ROS_UTILS_H_
#define _PBI_ROS_UTILS_H_

#include <string>

#include "ros/ros.h"

namespace pbi {
// GetParam wraps ros::param::get, logging an error message if the param is not
// found.
// Returns true if params is found, false otherwise.
bool GetParam(const std::string& name, float* val);
bool GetParam(const std::string& name, double* val);
bool GetParam(const std::string& name, bool* val);
bool GetParam(const std::string& name, int* val);

void LogGetParamFail(const std::string& name);
}  // namespace pbi

#endif  // _PBI_ROS_UTILS_H_
