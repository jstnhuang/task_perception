#ifndef _PBI_PR2_GRIPPER_MODEL_H_
#define _PBI_PR2_GRIPPER_MODEL_H_

#include <string>

#include "Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "transform_graph/graph.h"
#include "visualization_msgs/MarkerArray.h"

namespace pbi {
class Pr2GripperModel {
 public:
  Pr2GripperModel();
  void set_pose(const geometry_msgs::Pose& pose);

  // Appends three box markers representing our collision model for the gripper.
  void ToMarkerArray(const std::string& frame_id,
                     visualization_msgs::MarkerArray* marker_arr);

  // Returns the "center" of the gripper's grasp region.
  Eigen::Vector3d gripper_center();

 private:
  geometry_msgs::Pose pose_;

  transform_graph::Graph tf_graph_;

  const static geometry_msgs::Point kPalmPos;
  const static geometry_msgs::Point kLFingerPos;
  const static geometry_msgs::Point kRFingerPos;
  const static geometry_msgs::Vector3 kPalmDims;
  const static geometry_msgs::Vector3 kFingerDims;
};
}  // namespace pbi

#endif  // _PBI_PR2_GRIPPER_MODEL_H_
