#ifndef _PBI_PR2_GRIPPER_VIZ_H_
#define _PBI_PR2_GRIPPER_VIZ_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "visualization_msgs/MarkerArray.h"

namespace pbi {
class Pr2GripperViz {
 public:
  Pr2GripperViz();
  void set_frame_id(const std::string& frame_id);
  void set_pose(const geometry_msgs::Pose& pose);
  visualization_msgs::MarkerArray markers(const std::string& ns);

 private:
  void InitMarkers();

  visualization_msgs::MarkerArray markers_;
  std::string frame_id_;
  geometry_msgs::Pose pose_;
};
}  // namespace pbi

#endif  // _PBI_PR2_GRIPPER_VIZ_H_
