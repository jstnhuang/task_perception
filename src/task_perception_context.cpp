#include "task_perception/task_perception_context.h"

#include <map>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "absl/strings/str_cat.h"
#include "absl/strings/strip.h"
#include "eigen_conversions/eigen_msg.h"
#include "image_geometry/pinhole_camera_model.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/GetSkeletonState.h"
#include "skin_segmentation_msgs/PredictHands.h"
#include "task_perception_msgs/DemoState.h"

#include "task_perception/pcl_typedefs.h"
#include "task_perception/ros_utils.h"
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
      kPackagePath_(ros::package::getPath("object_meshes") + "/object_models/"),
      are_objects_indexed_(false),
      current_objects_(),
      prev_objects_(),
      object_models_(object_models),
      object_clouds_(),
      object_trees_(),
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
  const std::string& mesh_name = current_objects_[name].mesh_name;

  if (object_models_->find(mesh_name) == object_models_->end()) {
    std::string path = absl::StrCat(kPackagePath_, mesh_name);
    path = ReplaceObjWithPcd(path);
    PointCloudP::Ptr model = LoadModel(path);
    model->header.frame_id = camera_info_.header.frame_id;
    object_models_->insert(
        std::pair<string, PointCloudP::Ptr>(mesh_name, model));
  }
  return object_models_->at(mesh_name);
}

PointCloudP::Ptr TaskPerceptionContext::GetObjectCloud(const string& name) {
  if (object_clouds_.find(name) == object_clouds_.end()) {
    IndexObjects();
    const geometry_msgs::Pose& object_pose = current_objects_[name].pose;
    PointCloudP::Ptr object_model = GetObjectModel(name);
    Eigen::Affine3d object_transform;
    tf::poseMsgToEigen(object_pose, object_transform);
    PointCloudP::Ptr object_cloud(new PointCloudP);
    pcl::transformPointCloud(*object_model, *object_cloud, object_transform);
    object_cloud->header.frame_id = camera_info_.header.frame_id;
    object_clouds_[name] = object_cloud;
  }
  return object_clouds_[name];
}

KdTreeP::Ptr TaskPerceptionContext::GetObjectTree(const string& name) {
  if (object_trees_.find(name) == object_trees_.end()) {
    KdTreeP::Ptr tree(new pcl::KdTreeFLANN<PointP>);
    tree->setInputCloud(GetObjectCloud(name));
    object_trees_[name] = tree;
  }
  return object_trees_[name];
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
  for (const auto& obj : current_state_.object_states) {
    current_objects_[obj.name] = obj;
  }
  for (const auto& obj : prev_state_.object_states) {
    prev_objects_[obj.name] = obj;
  }
  are_objects_indexed_ = true;
}

PointCloudP::Ptr TaskPerceptionContext::LoadModel(const string& mesh_path) {
  PointCloudP::Ptr object_model(new PointCloudP);
  pcl::io::loadPCDFile(mesh_path, *object_model);
  ROS_INFO("Loaded mesh %s with %ld points", mesh_path.c_str(),
           object_model->size());
  return object_model;
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

pcl::KdTree<pcl::PointXYZ>::Ptr TaskPerceptionContext::LeftHandTree() {
  if (!left_hand_tree_) {
    left_hand_tree_.reset(new pcl::KdTreeFLANN<PointP>);
    if (left_hand_indices_->size() > 0) {
      left_hand_tree_->setInputCloud(both_hands_cloud_, left_hand_indices_);
    }
  }
  return left_hand_tree_;
}

pcl::KdTree<pcl::PointXYZ>::Ptr TaskPerceptionContext::RightHandTree() {
  if (!right_hand_tree_) {
    right_hand_tree_.reset(new pcl::KdTreeFLANN<PointP>);
    if (right_hand_indices_->size() > 0) {
      right_hand_tree_->setInputCloud(both_hands_cloud_, right_hand_indices_);
    }
  }
  return right_hand_tree_;
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

std::string ReplaceObjWithPcd(const std::string& path) {
  absl::string_view updated_path(path);
  absl::ConsumeSuffix(&updated_path, ".obj");
  return absl::StrCat(updated_path, ".pcd");
}
}  // namespace pbi
