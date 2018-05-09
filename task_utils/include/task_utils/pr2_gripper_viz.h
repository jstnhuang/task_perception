#ifndef _PBI_PR2_GRIPPER_VIZ_H_
#define _PBI_PR2_GRIPPER_VIZ_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "visualization_msgs/MarkerArray.h"

namespace pbi {
class Pr2GripperViz {
 public:
  Pr2GripperViz();
  void Init();
  visualization_msgs::MarkerArray markers(const std::string& ns,
                                          const geometry_msgs::Pose& pose,
                                          const std::string& frame_id) const;

 private:
  visualization_msgs::MarkerArray markers_;
};
}  // namespace pbi

#endif  // _PBI_PR2_GRIPPER_VIZ_H_
