#ifndef _PBI_POSE_UTILS_H_
#define _PBI_POSE_UTILS_H_

#include "geometry_msgs/Pose.h"

namespace pbi {
double LinearDistance(const geometry_msgs::Pose& start,
                      const geometry_msgs::Pose& end);
double AngularDistance(const geometry_msgs::Pose& start,
                       const geometry_msgs::Pose& end);
}  // namespace pbi

#endif  // _PBI_POSE_UTILS_H_
