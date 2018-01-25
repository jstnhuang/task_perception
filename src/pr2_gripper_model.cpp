#include "task_perception/pr2_gripper_model.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/MarkerArray.h"

namespace tg = transform_graph;

namespace pbi {
namespace {
geometry_msgs::Point PalmPos() {
  geometry_msgs::Point palm_pos;
  palm_pos.x = 0.078;
  return palm_pos;
}

geometry_msgs::Point LFingerPos() {
  geometry_msgs::Point pos;
  pos.x = 0.156;
  pos.y = 0.065;
  return pos;
}

geometry_msgs::Point RFingerPos() {
  geometry_msgs::Point pos;
  pos.x = 0.156;
  pos.y = -0.065;
  return pos;
}

geometry_msgs::Vector3 PalmDims() {
  geometry_msgs::Vector3 vec;
  vec.x = 0.096;
  vec.y = 0.15;
  vec.z = 0.06;
  return vec;
}

geometry_msgs::Vector3 FingerDims() {
  geometry_msgs::Vector3 vec;
  vec.x = 0.06;
  vec.y = 0.045;
  vec.z = 0.03;
  return vec;
}
}
const geometry_msgs::Point Pr2GripperModel::kPalmPos = PalmPos();
const geometry_msgs::Point Pr2GripperModel::kLFingerPos = LFingerPos();
const geometry_msgs::Point Pr2GripperModel::kRFingerPos = RFingerPos();
const geometry_msgs::Vector3 Pr2GripperModel::kPalmDims = PalmDims();
const geometry_msgs::Vector3 Pr2GripperModel::kFingerDims = FingerDims();

Pr2GripperModel::Pr2GripperModel() : pose_(), tf_graph_() {
  pose_.orientation.w = 1;
  tf_graph_.Add("palm", tg::RefFrame("gripper"),
                tg::Transform(kPalmPos, tg::Orientation()));
  tf_graph_.Add("l_finger", tg::RefFrame("gripper"),
                tg::Transform(kLFingerPos, tg::Orientation()));
  tf_graph_.Add("r_finger", tg::RefFrame("gripper"),
                tg::Transform(kRFingerPos, tg::Orientation()));

  Eigen::Vector3d center;  // In gripper frame.
  center << kLFingerPos.x, 0, 0;
  tf_graph_.Add("grasp center", tg::RefFrame("gripper"),
                tg::Transform(center, tg::Orientation()));
}

void Pr2GripperModel::set_pose(const geometry_msgs::Pose& pose) {
  pose_ = pose;
  tf_graph_.Add("gripper", tg::RefFrame("gripper base"), pose);
}

geometry_msgs::Pose Pr2GripperModel::pose() const { return pose_; }

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

  marker_arr->markers.push_back(palm);
  marker_arr->markers.push_back(l_finger);
  marker_arr->markers.push_back(r_finger);
}

Eigen::Vector3d Pr2GripperModel::grasp_center() const {
  tg::Transform grasp_center;
  tf_graph_.ComputeDescription(tg::LocalFrame("grasp center"),
                               tg::RefFrame("gripper base"), &grasp_center);
  return grasp_center.matrix().topRightCorner(3, 1);
}

const tg::Graph& Pr2GripperModel::tf_graph() const { return tf_graph_; }
}  // namespace pbi
