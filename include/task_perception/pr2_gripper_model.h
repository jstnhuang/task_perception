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
  geometry_msgs::Pose pose() const;

  // Appends three box markers representing our collision model for the gripper.
  void ToMarkerArray(const std::string& frame_id,
                     visualization_msgs::MarkerArray* marker_arr) const;

  // Returns the "center" of the gripper's grasp region.
  Eigen::Vector3d grasp_center() const;

  // Defines the following frames:
  // gripper base: The fixed frame (e.g., camera frame)
  // gripper: The gripper link
  // palm: The palm link
  // l_finger: The left finger link
  // r_finger: The right finger link
  // grasp center: The center of the grasp region (CoG)
  const transform_graph::Graph& tf_graph() const;

  // These functions assume that X/Y/Z have already been transformed into the
  // gripper frame, which differs from other methods in this class. This is
  // because we typically want to call both methods for the same point, so it's
  // better to transform all the points first.
  static bool IsGripperFramePtInGraspRegion(double x, double y, double z);
  static bool IsGripperFramePtInCollision(double x, double y, double z);

 private:
  geometry_msgs::Pose pose_;

  transform_graph::Graph tf_graph_;

  const static geometry_msgs::Point kPalmPos;
  const static geometry_msgs::Point kLFingerPos;
  const static geometry_msgs::Point kRFingerPos;
  const static geometry_msgs::Point kGraspRegionPos;
  const static geometry_msgs::Vector3 kPalmDims;
  const static geometry_msgs::Vector3 kFingerDims;
  const static geometry_msgs::Vector3 kGraspRegionDims;
};
}  // namespace pbi

#endif  // _PBI_PR2_GRIPPER_MODEL_H_
