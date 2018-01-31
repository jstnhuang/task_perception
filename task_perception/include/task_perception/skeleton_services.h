#ifndef _PBI_SKELETON_SERVICES_H_
#define _PBI_SKELETON_SERVICES_H_

#include "ros/ros.h"

namespace pbi {
// Bundles skeleton tracker services.
struct SkeletonServices {
  ros::ServiceClient reset;
  ros::ServiceClient advance;
  ros::ServiceClient get_state;
  ros::Publisher nerf_pub;
};
}  // namespace pbi

#endif  // _PBI_SKELETON_SERVICES_H_
