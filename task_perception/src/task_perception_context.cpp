#include "task_perception/task_perception_context.h"

#include <map>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "image_geometry/pinhole_camera_model.h"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/GetSkeletonState.h"
#include "skin_segmentation_msgs/PredictHands.h"
#include "task_perception_msgs/DemoState.h"
#include "task_utils/ros_params.h"

#include "task_perception/lazy_object_model.h"
#include "task_perception/pcl_typedefs.h"
#include "task_perception/skeleton_services.h"

using std::vector;
using std::string;
namespace msgs = task_perception_msgs;
namespace ss_msgs = skin_segmentation_msgs;

namespace pbi {
TaskPerceptionContext::TaskPerceptionContext(
    pbi::SkeletonServices& skel_services, ros::ServiceClient& predict_hands,
    const msgs::DemoState& current_state, const msgs::DemoState& prev_state,
    const sensor_msgs::Image& color_image,
    const sensor_msgs::Image& depth_image,
    const sensor_msgs::CameraInfo& camera_info,
    std::map<string, PointCloudP::Ptr>* object_models)
    : kDebug(false),
      kCloseToWristDistance(0),
      kMovingObjectDistance(0),
      kTouchingObjectDistance(0),
      kTouchingReleasedObjectDistance(0),
      kTouchingObjectPoints(0),
      kTouchingReleasedObjectPoints(0),
      kPartOfHandDistance(0),
      skel_services_(skel_services),
      predict_hands_(predict_hands),
      current_state_(current_state),
      prev_state_(prev_state),
      color_image_(color_image),
      depth_image_(depth_image),
      camera_info_(camera_info),
      have_wrist_poses_(false),
      left_wrist_pose_(),
      right_wrist_pose_(),
      are_objects_indexed_(false),
      current_objects_(),
      prev_objects_(),
      object_models_(object_models),
      lazy_objects_(),
      both_hands_cloud_(),
      left_hand_indices_(),
      right_hand_indices_(),
      left_hand_tree_(),
      right_hand_tree_() {}

bool TaskPerceptionContext::LoadParams() {
  if (!GetParam("contact_detection/debug", &kDebug) ||
      !GetParam("contact_detection/close_to_wrist_distance",
                &kCloseToWristDistance) ||
      !GetParam("contact_detection/moving_object_distance",
                &kMovingObjectDistance) ||
      !GetParam("contact_detection/touching_object_distance",
                &kTouchingObjectDistance) ||
      !GetParam("contact_detection/touching_released_object_distance",
                &kTouchingReleasedObjectDistance) ||
      !GetParam("contact_detection/touching_object_points",
                &kTouchingObjectPoints) ||
      !GetParam("contact_detection/touching_released_object_points",
                &kTouchingReleasedObjectPoints) ||
      !GetParam("contact_detection/part_of_hand_distance",
                &kPartOfHandDistance)) {
    return false;
  }
  return true;
}

const geometry_msgs::Pose& TaskPerceptionContext::GetLeftWristPose() {
  GetWristPoses();
  return left_wrist_pose_;
}

const geometry_msgs::Pose& TaskPerceptionContext::GetRightWristPose() {
  GetWristPoses();
  return right_wrist_pose_;
}

const vector<msgs::ObjectState>& TaskPerceptionContext::GetPrevObjects() const {
  return prev_state_.object_states;
}

const vector<msgs::ObjectState>& TaskPerceptionContext::GetCurrentObjects()
    const {
  return current_state_.object_states;
}

PointCloudP::Ptr TaskPerceptionContext::GetObjectModel(const string& name) {
  IndexObjects();
  return lazy_objects_.at(name).GetObjectModel();
}

PointCloudP::Ptr TaskPerceptionContext::GetObjectCloud(const string& name) {
  IndexObjects();
  PointCloudP::Ptr object_cloud = lazy_objects_.at(name).GetObjectCloud();
  object_cloud->header.frame_id = camera_info_.header.frame_id;
  return object_cloud;
}

PointCloudN::Ptr TaskPerceptionContext::GetObjectCloudWithNormals(
    const string& name) {
  IndexObjects();
  PointCloudN::Ptr output = lazy_objects_.at(name).GetObjectCloudWithNormals();
  output->header.frame_id = camera_info_.header.frame_id;
  return output;
}

KdTreeP::Ptr TaskPerceptionContext::GetObjectTree(const string& name) {
  IndexObjects();
  return lazy_objects_.at(name).GetObjectTree();
}

bool TaskPerceptionContext::GetCurrentObject(
    const std::string& name, task_perception_msgs::ObjectState* object) {
  IndexObjects();
  if (current_objects_.find(name) == current_objects_.end()) {
    return false;
  }
  *object = current_objects_[name];
  return true;
}

bool TaskPerceptionContext::GetPreviousObject(
    const std::string& name, task_perception_msgs::ObjectState* prev_obj) {
  IndexObjects();
  if (prev_objects_.find(name) == prev_objects_.end()) {
    return false;
  }
  *prev_obj = prev_objects_[name];
  return true;
}

void TaskPerceptionContext::GetWristPoses() {
  if (!have_wrist_poses_) {
    ss_msgs::GetSkeletonStateRequest state_req;
    ss_msgs::GetSkeletonStateResponse state_res;
    skel_services_.get_state.call(state_req, state_res);
    left_wrist_pose_ = state_res.left_wrist;
    right_wrist_pose_ = state_res.right_wrist;
    have_wrist_poses_ = true;
  }
}

void TaskPerceptionContext::IndexObjects() {
  if (are_objects_indexed_) {
    return;
  }
  for (size_t i = 0; i < current_state_.object_states.size(); ++i) {
    const msgs::ObjectState& obj = current_state_.object_states[i];
    current_objects_[obj.name] = obj;
    LazyObjectModel lazy_model(obj.name, obj.mesh_name,
                               camera_info_.header.frame_id, obj.pose);
    lazy_model.set_object_model_cache(object_models_);
    lazy_objects_.insert(
        std::pair<std::string, LazyObjectModel>(obj.name, lazy_model));
  }
  for (size_t i = 0; i < prev_state_.object_states.size(); ++i) {
    const msgs::ObjectState& obj = prev_state_.object_states[i];
    prev_objects_[obj.name] = obj;
  }
  are_objects_indexed_ = true;
}

ros::Time TaskPerceptionContext::GetPreviousTime() { return prev_state_.stamp; }

ros::Time TaskPerceptionContext::GetCurrentTime() {
  return current_state_.stamp;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TaskPerceptionContext::BothHandsCloud() {
  if (!both_hands_cloud_) {
    ComputeHandClouds();
  }
  return both_hands_cloud_;
}

pcl::IndicesPtr TaskPerceptionContext::LeftHandIndices() {
  if (!left_hand_indices_) {
    ComputeHandClouds();
  }
  return left_hand_indices_;
}

pcl::IndicesPtr TaskPerceptionContext::RightHandIndices() {
  if (!right_hand_indices_) {
    ComputeHandClouds();
  }
  return right_hand_indices_;
}

KdTreeP::Ptr TaskPerceptionContext::LeftHandTree() {
  if (!left_hand_tree_) {
    left_hand_tree_.reset(new pcl::search::KdTree<PointP>);
    if (left_hand_indices_->size() > 0) {
      left_hand_tree_->setInputCloud(both_hands_cloud_, left_hand_indices_);
    }
  }
  return left_hand_tree_;
}

KdTreeP::Ptr TaskPerceptionContext::RightHandTree() {
  if (!right_hand_tree_) {
    right_hand_tree_.reset(new pcl::search::KdTree<PointP>);
    if (right_hand_indices_->size() > 0) {
      right_hand_tree_->setInputCloud(both_hands_cloud_, right_hand_indices_);
    }
  }
  return right_hand_tree_;
}

const msgs::DemoState& TaskPerceptionContext::current_state() const {
  return current_state_;
}

const msgs::DemoState& TaskPerceptionContext::prev_state() const {
  return prev_state_;
}

const sensor_msgs::CameraInfo& TaskPerceptionContext::camera_info() const {
  return camera_info_;
}

// EXPECTED POST-CONDITION: both_hands_cloud_, left_hand_indices_, and
// right_hand_indices_ must not be null.
void TaskPerceptionContext::ComputeHandClouds() {
  if (both_hands_cloud_) {
    return;
  }

  // Find hand pixels
  ss_msgs::PredictHandsRequest req;
  req.rgb = color_image_;
  req.depth_registered = depth_image_;
  ss_msgs::PredictHandsResponse res;
  predict_hands_.call(req, res);
  const sensor_msgs::Image& hands = res.prediction;

  both_hands_cloud_.reset(new PointCloudP);
  left_hand_indices_.reset(new std::vector<int>());
  right_hand_indices_.reset(new std::vector<int>());
  if (hands.encoding != "mono8") {
    ROS_ERROR("Unsupported hand prediction format: %s", hands.encoding.c_str());
    return;
  }

  const uint16_t* depth_16;
  const float* depth_float;
  bool is_float = depth_image_.encoding == "32FC1";
  if (is_float) {
    depth_float = reinterpret_cast<const float*>(depth_image_.data.data());
  } else if (depth_image_.encoding == "16UC1") {
    depth_16 = reinterpret_cast<const uint16_t*>(depth_image_.data.data());
  } else {
    ROS_ERROR("Unsupported depth image type: %s",
              depth_image_.encoding.c_str());
    return;
  }

  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info_);
  for (unsigned int row = 0; row < hands.height; ++row) {
    for (unsigned int col = 0; col < hands.width; ++col) {
      int index = row * hands.step + col;

      float depth_meters = 0;
      if (is_float) {
        depth_meters = depth_float[index];
      } else {
        depth_meters = depth_16[index] / 1000.0;
      }
      if (hands.data[index] > 0 && depth_meters > 0) {
        cv::Point3d ray =
            camera_model.projectPixelTo3dRay(cv::Point2d(col, row));
        ray *= depth_meters;
        PointP pt;
        pt.x = ray.x;
        pt.y = ray.y;
        pt.z = ray.z;
        both_hands_cloud_->push_back(pt);
      }
    }
  }
  both_hands_cloud_->header.frame_id = camera_info_.header.frame_id;

  if (both_hands_cloud_->size() == 0) {
    ROS_WARN("Hands not found in the scene");
  }

  ComputeLeftRightHands();
}

// Precondition: both_hands_clouds_ must have been initialized.
// left_hand_indices_ and right_hand_indices_ must not be null.
void TaskPerceptionContext::ComputeLeftRightHands() {
  const geometry_msgs::Pose& left = GetLeftWristPose();
  const geometry_msgs::Pose& right = GetRightWristPose();
  const float kSqMaxDistance = kPartOfHandDistance * kPartOfHandDistance;
  for (size_t i = 0; i < both_hands_cloud_->size(); ++i) {
    const PointP& pt = both_hands_cloud_->points[i];
    float dx = pt.x - left.position.x;
    float dy = pt.y - left.position.y;
    float dz = pt.z - left.position.z;
    float left_sq_dist = dx * dx + dy * dy + dz * dz;
    dx = pt.x - right.position.x;
    dy = pt.y - right.position.y;
    dz = pt.z - right.position.z;
    float right_sq_dist = dx * dx + dy * dy + dz * dz;

    if (left_sq_dist >= kSqMaxDistance && right_sq_dist >= kSqMaxDistance) {
      // If point is far from both, skip.
      continue;
    } else if (left_sq_dist < kSqMaxDistance &&
               right_sq_dist < kSqMaxDistance) {
      // If point is close enough to both left and right, then associate with
      // the closer hand.
      if (left_sq_dist < right_sq_dist) {
        left_hand_indices_->push_back(i);
      } else {
        right_hand_indices_->push_back(i);
      }
      continue;
    } else if (left_sq_dist < kSqMaxDistance &&
               right_sq_dist >= kSqMaxDistance) {
      left_hand_indices_->push_back(i);
    } else {
      right_hand_indices_->push_back(i);
    }
  }
}
}  // namespace pbi
