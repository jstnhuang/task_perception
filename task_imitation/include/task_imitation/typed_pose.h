#ifndef _PBI_TYPED_POSE_H_
#define _PBI_TYPED_POSE_H_

#include <string>

#include "geometry_msgs/Pose.h"

namespace pbi {
struct TypedPose {
 public:
  enum Type { PREGRASP, GRASP, MOVE_TO, TRAJECTORY };
  geometry_msgs::Pose pose;
  Type type;
};
}  // namespace pbi

#endif  // _PBI_TYPED_POSE_H_
