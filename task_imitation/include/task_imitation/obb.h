#ifndef _PBI_OBB_H_
#define _PBI_OBB_H_

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace pbi {
// An oriented bounding box
struct Obb {
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 dims;
};
}  // namespace pbi

#endif  // _PBI_OBB_H_
