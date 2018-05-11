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

  // Returns the "center" of the gripper's grasp region in the world frame.
  Eigen::Vector3d grasp_center() const;
  // Returns the center of the grasp, but farther forward (about 1 cm away from
  // the edge of the grasp region)
  Eigen::Vector3d forward_grasp_center() const;

  // Returns the center of the palm in the world frame.
  // This can be a good proxy for a human wrist.
  Eigen::Vector3d palm_center() const;

  // Defines the following frames:
  // gripper base: The world frame (e.g., camera frame or planning frame)
  // gripper: The gripper link
  // palm: The palm link
  // l_finger: The left finger link
  // r_finger: The right finger link
  // grasp center: The center of the grasp region (CoG)
  const transform_graph::Graph& tf_graph() const;

  bool IsCollidingWithObb(const geometry_msgs::Pose& pose,
                          const geometry_msgs::Vector3& dims) const;
  // Returns NONE, PALM, GRASP_REGION, L_FINGER, R_FINGER. Checks in that order
  // and returns immediately on the first collision.
  int CheckCollisionWithObb(const geometry_msgs::Pose& pose,
                            const geometry_msgs::Vector3& dims) const;

  // These functions assume that X/Y/Z have already been transformed into the
  // gripper frame, which differs from other methods in this class. This is
  // because we typically want to call both methods for the same point, so it's
  // better to transform all the points first.
  static bool IsGripperFramePtInGraspRegion(double x, double y, double z);
  static bool IsGripperFramePtInCollision(double x, double y, double z);

  const static geometry_msgs::Point kPalmPos;
  const static geometry_msgs::Point kLFingerPos;
  const static geometry_msgs::Point kRFingerPos;
  const static geometry_msgs::Point kGraspRegionPos;
  const static geometry_msgs::Vector3 kPalmDims;
  const static geometry_msgs::Vector3 kFingerDims;
  const static geometry_msgs::Vector3 kGraspRegionDims;
  const static int NONE = 0;
  const static int PALM = 1;
  const static int GRASP_REGION = 2;
  const static int L_FINGER = 3;
  const static int R_FINGER = 4;

 private:
  geometry_msgs::Pose pose_;

  transform_graph::Graph tf_graph_;
};
}  // namespace pbi

#endif  // _PBI_PR2_GRIPPER_MODEL_H_
