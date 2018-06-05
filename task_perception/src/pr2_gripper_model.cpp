#include "task_perception/pr2_gripper_model.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "rapid_collision/collision_checks.h"
#include "visualization_msgs/MarkerArray.h"

namespace tg = transform_graph;
using geometry_msgs::Point;
using geometry_msgs::Pose;
using geometry_msgs::Vector3;

namespace pbi {
namespace {
Point PalmPos() {
  Point palm_pos;
  palm_pos.x = 0.078;
  return palm_pos;
}

Point LFingerPos() {
  Point pos;
  pos.x = 0.156;
  pos.y = 0.065 - 0.01225;
  return pos;
}

Point RFingerPos() {
  Point pos;
  pos.x = 0.156;
  pos.y = -0.065 + 0.01225;
  return pos;
}

Point LKnucklePos() {
  Point pos;
  pos.x = 0.156 - 0.015;
  pos.y = 0.065 + 0.01125;
  return pos;
}

Point RKnucklePos() {
  Point pos;
  pos.x = 0.156 - 0.015;
  pos.y = -0.065 - 0.01125;
  return pos;
}

Point GraspRegionPos() {
  Point pos;
  pos.x = 0.1625;
  return pos;
}

Vector3 PalmDims() {
  Vector3 vec;
  vec.x = 0.096;
  vec.y = 0.15;
  vec.z = 0.06;
  return vec;
}

Vector3 FingerDims() {
  Vector3 vec;
  vec.x = 0.06;
  vec.y = 0.0205;  // 0.0225;
  vec.z = 0.023;
  return vec;
}

Vector3 KnuckleDims() {
  Vector3 vec;
  vec.x = 0.03;
  vec.y = 0.0225;
  vec.z = 0.026;
  return vec;
}

Vector3 GraspRegionDims() {
  Vector3 vec;
  vec.x = 0.073;
  vec.y = 0.085;
  vec.z = 0.025;
  return vec;
}
}  // namespace
const Point Pr2GripperModel::kPalmPos = PalmPos();
const Point Pr2GripperModel::kLFingerPos = LFingerPos();
const Point Pr2GripperModel::kRFingerPos = RFingerPos();
const Point Pr2GripperModel::kLKnucklePos = LKnucklePos();
const Point Pr2GripperModel::kRKnucklePos = RKnucklePos();
const Point Pr2GripperModel::kGraspRegionPos = GraspRegionPos();
const Vector3 Pr2GripperModel::kPalmDims = PalmDims();
const Vector3 Pr2GripperModel::kFingerDims = FingerDims();
const Vector3 Pr2GripperModel::kKnuckleDims = KnuckleDims();
const Vector3 Pr2GripperModel::kGraspRegionDims = GraspRegionDims();

Pr2GripperModel::Pr2GripperModel() : pose_(), tf_graph_() {
  pose_.orientation.w = 1;
  tf_graph_.Add("palm", tg::RefFrame("gripper"),
                tg::Transform(kPalmPos, tg::Orientation()));
  tf_graph_.Add("l_finger", tg::RefFrame("gripper"),
                tg::Transform(kLFingerPos, tg::Orientation()));
  tf_graph_.Add("r_finger", tg::RefFrame("gripper"),
                tg::Transform(kRFingerPos, tg::Orientation()));
  tf_graph_.Add("l_knuckle", tg::RefFrame("gripper"),
                tg::Transform(kLKnucklePos, tg::Orientation()));
  tf_graph_.Add("r_knuckle", tg::RefFrame("gripper"),
                tg::Transform(kRKnucklePos, tg::Orientation()));

  Eigen::Vector3d center;  // In gripper frame.
  center << kGraspRegionPos.x, kGraspRegionPos.y, kGraspRegionPos.z;
  tf_graph_.Add("grasp center", tg::RefFrame("gripper"),
                tg::Transform(center, tg::Orientation()));
}

void Pr2GripperModel::set_pose(const Pose& pose) {
  pose_ = pose;
  tf_graph_.Add("gripper", tg::RefFrame("gripper base"), pose);
}

Pose Pr2GripperModel::pose() const { return pose_; }

void Pr2GripperModel::ToMarkerArray(
    const std::string& frame_id,
    visualization_msgs::MarkerArray* marker_arr) const {
  visualization_msgs::Marker palm;
  palm.header.frame_id = frame_id;
  palm.ns = "gripper_model";
  palm.id = 0;
  palm.type = visualization_msgs::Marker::CUBE;
  palm.scale = kPalmDims;
  palm.color.r = 1;
  palm.color.g = 1;
  palm.color.b = 1;
  palm.color.a = 0.5;

  tg::Transform palm_transform;
  tf_graph_.ComputeDescription(tg::LocalFrame("palm"),
                               tg::RefFrame("gripper base"), &palm_transform);
  palm_transform.ToPose(&palm.pose);

  visualization_msgs::Marker l_finger;
  l_finger.header.frame_id = frame_id;
  l_finger.ns = "gripper_model";
  l_finger.id = 1;
  l_finger.type = visualization_msgs::Marker::CUBE;
  l_finger.scale = kFingerDims;
  l_finger.color.r = 1;
  l_finger.color.g = 1;
  l_finger.color.b = 1;
  l_finger.color.a = 0.5;

  tg::Transform l_finger_transform;
  tf_graph_.ComputeDescription(tg::LocalFrame("l_finger"),
                               tg::RefFrame("gripper base"),
                               &l_finger_transform);
  l_finger_transform.ToPose(&l_finger.pose);

  visualization_msgs::Marker r_finger;
  r_finger.header.frame_id = frame_id;
  r_finger.ns = "gripper_model";
  r_finger.id = 2;
  r_finger.type = visualization_msgs::Marker::CUBE;
  r_finger.scale = kFingerDims;
  r_finger.color.r = 1;
  r_finger.color.g = 1;
  r_finger.color.b = 1;
  r_finger.color.a = 0.5;

  tg::Transform r_finger_transform;
  tf_graph_.ComputeDescription(tg::LocalFrame("r_finger"),
                               tg::RefFrame("gripper base"),
                               &r_finger_transform);
  r_finger_transform.ToPose(&r_finger.pose);

  visualization_msgs::Marker l_knuckle;
  l_knuckle.header.frame_id = frame_id;
  l_knuckle.ns = "gripper_model";
  l_knuckle.id = 3;
  l_knuckle.type = visualization_msgs::Marker::CUBE;
  l_knuckle.scale = kKnuckleDims;
  l_knuckle.color.r = 1;
  l_knuckle.color.g = 1;
  l_knuckle.color.b = 1;
  l_knuckle.color.a = 0.5;

  tg::Transform l_knuckle_transform;
  tf_graph_.ComputeDescription(tg::LocalFrame("l_knuckle"),
                               tg::RefFrame("gripper base"),
                               &l_knuckle_transform);
  l_knuckle_transform.ToPose(&l_knuckle.pose);

  visualization_msgs::Marker r_knuckle;
  r_knuckle.header.frame_id = frame_id;
  r_knuckle.ns = "gripper_model";
  r_knuckle.id = 4;
  r_knuckle.type = visualization_msgs::Marker::CUBE;
  r_knuckle.scale = kKnuckleDims;
  r_knuckle.color.r = 1;
  r_knuckle.color.g = 1;
  r_knuckle.color.b = 1;
  r_knuckle.color.a = 0.5;

  tg::Transform r_knuckle_transform;
  tf_graph_.ComputeDescription(tg::LocalFrame("r_knuckle"),
                               tg::RefFrame("gripper base"),
                               &r_knuckle_transform);
  r_knuckle_transform.ToPose(&r_knuckle.pose);

  visualization_msgs::Marker grasp_region;
  grasp_region.header.frame_id = frame_id;
  grasp_region.ns = "gripper_model";
  grasp_region.id = 5;
  grasp_region.type = visualization_msgs::Marker::CUBE;
  grasp_region.scale = kGraspRegionDims;
  grasp_region.color.r = 0.5;
  grasp_region.color.g = 1;
  grasp_region.color.b = 0.5;
  grasp_region.color.a = 0.2;

  tg::Transform grasp_region_transform;
  tf_graph_.ComputeDescription(tg::LocalFrame("grasp center"),
                               tg::RefFrame("gripper base"),
                               &grasp_region_transform);
  grasp_region_transform.ToPose(&grasp_region.pose);

  marker_arr->markers.push_back(palm);
  marker_arr->markers.push_back(l_finger);
  marker_arr->markers.push_back(r_finger);
  marker_arr->markers.push_back(l_knuckle);
  marker_arr->markers.push_back(r_knuckle);
  marker_arr->markers.push_back(grasp_region);
}

Eigen::Vector3d Pr2GripperModel::grasp_center() const {
  tg::Transform grasp_center;
  tf_graph_.ComputeDescription(tg::LocalFrame("grasp center"),
                               tg::RefFrame("gripper base"), &grasp_center);
  return grasp_center.matrix().topRightCorner(3, 1);
}

Eigen::Vector3d Pr2GripperModel::forward_grasp_center(double obj_width) const {
  if (obj_width < 0) {
    obj_width = 0;
  }
  if (obj_width > kGraspRegionDims.y) {
    obj_width = kGraspRegionDims.y;
  }
  double most_forward_x = kGraspRegionDims.x / 2 - 0.005;
  double least_forward_x = kGraspRegionDims.x / 2 - 0.023;
  double slope = (least_forward_x - most_forward_x) / kGraspRegionDims.y;
  double forward_offset = slope * obj_width + most_forward_x;

  tg::Position forward_pos;
  tf_graph_.DescribePosition(tg::Position(forward_offset, 0, 0),
                             tg::Source("grasp center"),
                             tg::Target("gripper base"), &forward_pos);
  return forward_pos.vector();
}

Eigen::Vector3d Pr2GripperModel::palm_center() const {
  tg::Transform palm_center;
  tf_graph_.ComputeDescription("palm", tg::RefFrame("gripper base"),
                               &palm_center);
  return palm_center.position().vector();
}

const tg::Graph& Pr2GripperModel::tf_graph() const { return tf_graph_; }

bool Pr2GripperModel::IsCollidingWithObb(const Pose& pose,
                                         const Vector3& dims) const {
  return CheckCollisionWithObb(pose, dims) != NONE;
}

int Pr2GripperModel::CheckCollisionWithObb(const Pose& pose,
                                           const Vector3& dims) const {
  tg::Transform palm_tf;
  tf_graph_.ComputeDescription("palm", tg::RefFrame("gripper base"), &palm_tf);
  if (rapid::AreObbsInCollision(pose, dims, palm_tf.pose(), kPalmDims)) {
    return PALM;
  }

  tg::Transform grasp_tf;
  tf_graph_.ComputeDescription("grasp center", tg::RefFrame("gripper base"),
                               &grasp_tf);
  if (rapid::AreObbsInCollision(pose, dims, grasp_tf.pose(),
                                kGraspRegionDims)) {
    return GRASP_REGION;
  }

  tg::Transform l_finger_tf;
  tf_graph_.ComputeDescription("l_finger", tg::RefFrame("gripper base"),
                               &l_finger_tf);
  if (rapid::AreObbsInCollision(pose, dims, l_finger_tf.pose(), kFingerDims)) {
    return L_FINGER;
  }

  tg::Transform r_finger_tf;
  tf_graph_.ComputeDescription("r_finger", tg::RefFrame("gripper base"),
                               &r_finger_tf);
  if (rapid::AreObbsInCollision(pose, dims, r_finger_tf.pose(), kFingerDims)) {
    return R_FINGER;
  }

  tg::Transform l_knuckle_tf;
  tf_graph_.ComputeDescription("l_knuckle", tg::RefFrame("gripper base"),
                               &l_knuckle_tf);
  if (rapid::AreObbsInCollision(pose, dims, l_knuckle_tf.pose(),
                                kKnuckleDims)) {
    return L_KNUCKLE;
  }

  tg::Transform r_knuckle_tf;
  tf_graph_.ComputeDescription("r_knuckle", tg::RefFrame("gripper base"),
                               &r_knuckle_tf);
  if (rapid::AreObbsInCollision(pose, dims, r_knuckle_tf.pose(),
                                kKnuckleDims)) {
    return R_KNUCKLE;
  }

  return NONE;
}

bool Pr2GripperModel::IsGripperFramePtInGraspRegion(double x, double y,
                                                    double z) {
  double max_grasp_region_x = kGraspRegionPos.x + kGraspRegionDims.x / 2;
  double min_grasp_region_x = kGraspRegionPos.x - kGraspRegionDims.x / 2;
  double max_grasp_region_y = kGraspRegionPos.y + kGraspRegionDims.y / 2;
  double min_grasp_region_y = kGraspRegionPos.y - kGraspRegionDims.y / 2;
  double max_grasp_region_z = kGraspRegionPos.z + kGraspRegionDims.z / 2;
  double min_grasp_region_z = kGraspRegionPos.z - kGraspRegionDims.z / 2;
  if (x < max_grasp_region_x && x > min_grasp_region_x &&
      y < max_grasp_region_y && y > min_grasp_region_y &&
      z < max_grasp_region_z && z > min_grasp_region_z) {
    return true;
  }

  return false;
}

bool Pr2GripperModel::IsGripperFramePtInCollision(double x, double y,
                                                  double z) {
  double max_palm_x = kPalmPos.x + kPalmDims.x / 2;
  double min_palm_x = kPalmPos.x - kPalmDims.x / 2;
  double max_palm_y = kPalmPos.y + kPalmDims.y / 2;
  double min_palm_y = kPalmPos.y - kPalmDims.y / 2;
  double max_palm_z = kPalmPos.z + kPalmDims.z / 2;
  double min_palm_z = kPalmPos.z - kPalmDims.z / 2;
  if (x < max_palm_x && x > min_palm_x && y < max_palm_y && y > min_palm_y &&
      z < max_palm_z && z > min_palm_z) {
    return true;
  }

  double max_l_finger_x = kLFingerPos.x + kFingerDims.x / 2;
  double min_l_finger_x = kLFingerPos.x - kFingerDims.x / 2;
  double max_l_finger_y = kLFingerPos.y + kFingerDims.y / 2;
  double min_l_finger_y = kLFingerPos.y - kFingerDims.y / 2;
  double max_l_finger_z = kLFingerPos.z + kFingerDims.z / 2;
  double min_l_finger_z = kLFingerPos.z - kFingerDims.z / 2;
  if (x < max_l_finger_x && x > min_l_finger_x && y < max_l_finger_y &&
      y > min_l_finger_y && z < max_l_finger_z && z > min_l_finger_z) {
    return true;
  }

  double max_r_finger_x = kRFingerPos.x + kFingerDims.x / 2;
  double min_r_finger_x = kRFingerPos.x - kFingerDims.x / 2;
  double max_r_finger_y = kRFingerPos.y + kFingerDims.y / 2;
  double min_r_finger_y = kRFingerPos.y - kFingerDims.y / 2;
  double max_r_finger_z = kRFingerPos.z + kFingerDims.z / 2;
  double min_r_finger_z = kRFingerPos.z - kFingerDims.z / 2;
  if (x < max_r_finger_x && x > min_r_finger_x && y < max_r_finger_y &&
      y > min_r_finger_y && z < max_r_finger_z && z > min_r_finger_z) {
    return true;
  }

  double max_l_knuckle_x = kLKnucklePos.x + kKnuckleDims.x / 2;
  double min_l_knuckle_x = kLKnucklePos.x - kKnuckleDims.x / 2;
  double max_l_knuckle_y = kLKnucklePos.y + kKnuckleDims.y / 2;
  double min_l_knuckle_y = kLKnucklePos.y - kKnuckleDims.y / 2;
  double max_l_knuckle_z = kLKnucklePos.z + kKnuckleDims.z / 2;
  double min_l_knuckle_z = kLKnucklePos.z - kKnuckleDims.z / 2;
  if (x < max_l_knuckle_x && x > min_l_knuckle_x && y < max_l_knuckle_y &&
      y > min_l_knuckle_y && z < max_l_knuckle_z && z > min_l_knuckle_z) {
    return true;
  }

  double max_r_knuckle_x = kRKnucklePos.x + kKnuckleDims.x / 2;
  double min_r_knuckle_x = kRKnucklePos.x - kKnuckleDims.x / 2;
  double max_r_knuckle_y = kRKnucklePos.y + kKnuckleDims.y / 2;
  double min_r_knuckle_y = kRKnucklePos.y - kKnuckleDims.y / 2;
  double max_r_knuckle_z = kRKnucklePos.z + kKnuckleDims.z / 2;
  double min_r_knuckle_z = kRKnucklePos.z - kKnuckleDims.z / 2;
  if (x < max_r_knuckle_x && x > min_r_knuckle_x && y < max_r_knuckle_y &&
      y > min_r_knuckle_y && z < max_r_knuckle_z && z > min_r_knuckle_z) {
    return true;
  }

  return false;
}
}  // namespace pbi
