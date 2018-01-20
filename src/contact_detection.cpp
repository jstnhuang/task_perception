#include "task_perception/contact_detection.h"

#include <math.h>
#include <algorithm>
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
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "skin_segmentation_msgs/GetSkeletonState.h"
#include "skin_segmentation_msgs/PredictHands.h"
#include "task_perception_msgs/DemoState.h"
#include "visualization_msgs/Marker.h"

#include "task_perception/pcl_typedefs.h"
#include "task_perception/pcl_utils.h"
#include "task_perception/pose_utils.h"
#include "task_perception/ros_utils.h"
#include "task_perception/skeleton_services.h"

using std::vector;
using std::string;
namespace msgs = task_perception_msgs;
namespace ss_msgs = skin_segmentation_msgs;

namespace pbi {
ContactDetection::ContactDetection(
    const SkeletonServices& skel_services,
    const ros::ServiceClient& predict_hands,
    std::map<string, PointCloudP::Ptr>* object_models)
    : skel_services_(skel_services),
      predict_hands_(predict_hands),
      object_models_(object_models),
      nh_(),
      viz_(nh_.advertise<visualization_msgs::Marker>(
          "contact_detection/markers", 10)),
      obj_viz_(nh_.advertise<sensor_msgs::PointCloud2>(
          "contact_detection/object_clouds", 1, true)),
      left_hand_viz_(nh_.advertise<sensor_msgs::PointCloud2>(
          "contact_detection/left_hand", 1, true)),
      right_hand_viz_(nh_.advertise<sensor_msgs::PointCloud2>(
          "contact_detection/right_hand", 1, true)) {}

void ContactDetection::Predict(const msgs::DemoState& current_state,
                               const msgs::DemoState& prev_state,
                               const sensor_msgs::Image& color_image,
                               const sensor_msgs::Image& depth_image,
                               const sensor_msgs::CameraInfo& camera_info,
                               msgs::HandState* left_hand,
                               msgs::HandState* right_hand) {
  ContactDetectionContext context(skel_services_, predict_hands_, current_state,
                                  prev_state, color_image, depth_image,
                                  camera_info, object_models_);
  if (!context.LoadParams()) {
    return;
  }

  PredictHandState(prev_state.left_hand, "left", &context, left_hand);
  PredictHandState(prev_state.right_hand, "right", &context, right_hand);

  if (context.kDebug) {
    geometry_msgs::Pose left_wrist = context.GetLeftWristPose();
    geometry_msgs::Pose right_wrist = context.GetRightWristPose();
    PublishWristPoses(left_wrist, right_wrist, camera_info.header.frame_id);
  }

  // Create point cloud for hands
  if (context.kDebug) {
    PointCloudP::Ptr hand_cloud = context.BothHandsCloud();
    PublishPointCloud(left_hand_viz_, hand_cloud, context.LeftHandIndices());
    PublishPointCloud(right_hand_viz_, hand_cloud, context.RightHandIndices());
  }
}

void ContactDetection::PredictHandState(const msgs::HandState& prev_state,
                                        const std::string& left_or_right,
                                        ContactDetectionContext* context,
                                        msgs::HandState* hand_state) {
  if (prev_state.current_action == "" ||
      prev_state.current_action == msgs::HandState::NONE) {
    CheckGrasp(prev_state, left_or_right, context, hand_state);
  } else if (prev_state.current_action == msgs::HandState::GRASPING) {
    CheckRelease(prev_state, left_or_right, context, hand_state);
  }
}

void ContactDetection::CheckGrasp(const msgs::HandState& prev_state,
                                  const std::string& left_or_right,
                                  ContactDetectionContext* context,
                                  msgs::HandState* hand_state) {
  // For all nearby objects:
  // - If the object is moving, set state to GRASPING
  // - If enough object points are close to hand points, set state to GRASPING
  // Otherwise, keep state as NONE
  geometry_msgs::Pose wrist_pose;
  if (left_or_right == "left") {
    wrist_pose = context->GetLeftWristPose();
  } else {
    wrist_pose = context->GetRightWristPose();
  }

  vector<msgs::ObjectState> current_objects = context->GetCurrentObjects();
  int num_touched_points = 0;
  for (const msgs::ObjectState& object : current_objects) {
    PointCloudP::Ptr object_cloud = context->GetObjectCloud(object.name);
    PublishPointCloud(obj_viz_, *object_cloud);

    if (!IsObjectCurrentlyCloseToWrist(wrist_pose, object.name, context)) {
      continue;
    }

    bool is_moving = IsObjectMoving(object, context);

    bool is_touching = false;
    if (context->BothHandsCloud()->size() == 0) {
      continue;
    }
    int num_touching_points = NumHandPointsOnObject(
        object, left_or_right, context, context->kTouchingObjectDistance);
    is_touching = num_touching_points >= context->kTouchingObjectPoints;

    if (is_moving || is_touching) {
      if (num_touching_points <= num_touched_points) {
        continue;
      }
      num_touched_points = num_touching_points;

      ROS_INFO("Changed %s hand state to GRASPING %s", left_or_right.c_str(),
               object.name.c_str());
      hand_state->current_action = msgs::HandState::GRASPING;
      hand_state->object_name = object.name;
      num_touched_points = num_touching_points;
      // TODO: fill in contact transform
    }
  }
  if (hand_state->current_action == "") {
    hand_state->current_action = msgs::HandState::NONE;
  }
}

void ContactDetection::CheckRelease(const msgs::HandState& prev_state,
                                    const std::string& left_or_right,
                                    ContactDetectionContext* context,
                                    msgs::HandState* hand_state) {
  // If object does not exist anymore, set state to NONE
  msgs::ObjectState object;
  if (!context->GetCurrentObject(prev_state.object_name, &object)) {
    ROS_WARN("Object \"%s\" disappeared while being contacted by %s hand.",
             prev_state.object_name.c_str(), left_or_right.c_str());
    hand_state->current_action = msgs::HandState::NONE;
    hand_state->object_name = "";
    // TODO: clear contact transform
    return;
  }

  // If not enough object points are close to hand points, set state to NONE
  // When we check for a release, we use more conservative parameters, since
  // sometimes the hand segmentation can flicker or the hand can be occluded
  // while grasping.
  if (context->BothHandsCloud()->size() == 0) {
    ROS_WARN(
        "Hands not found! Maintaining status quo, but this should not happen.");
    *hand_state = prev_state;
    return;
  }
  int num_touching_points = NumHandPointsOnObject(
      object, left_or_right, context, context->kTouchingReleasedObjectDistance);
  if (num_touching_points <= context->kTouchingReleasedObjectPoints) {
    ROS_INFO("Changed %s hand state to NONE (%d out of %d points)",
             left_or_right.c_str(), num_touching_points,
             context->kTouchingReleasedObjectPoints);
    hand_state->current_action = msgs::HandState::NONE;
    hand_state->object_name = "";
    // TODO: clear contact transform
    return;
  }

  // Otherwise, keep as GRASPING
  hand_state->current_action = msgs::HandState::GRASPING;
  hand_state->object_name = prev_state.object_name;
  // TODO: fill in contact transform
  return;
}

bool ContactDetection::IsObjectCurrentlyCloseToWrist(
    const geometry_msgs::Pose& wrist, const std::string& object_name,
    ContactDetectionContext* context) const {
  KdTreeP::Ptr object_tree = context->GetObjectTree(object_name);

  PointP wrist_point;
  wrist_point.x = wrist.position.x;
  wrist_point.y = wrist.position.y;
  wrist_point.z = wrist.position.z;
  vector<int> indices(1);
  vector<float> sq_distances(1);
  object_tree->nearestKSearch(wrist_point, 1, indices, sq_distances);
  float wrist_threshold =
      context->kCloseToWristDistance * context->kCloseToWristDistance;
  return sq_distances[0] <= wrist_threshold;
}

int ContactDetection::NumHandPointsOnObject(
    const task_perception_msgs::ObjectState& object,
    const std::string& left_or_right, ContactDetectionContext* context,
    const float distance_threshold) const {
  KdTreeP::Ptr object_tree = context->GetObjectTree(object.name);

  PointCloudP::Ptr both_hands_cloud = context->BothHandsCloud();
  pcl::IndicesPtr hand_indices;
  if (left_or_right == "left") {
    hand_indices = context->LeftHandIndices();
  } else {
    hand_indices = context->RightHandIndices();
  }

  int num_touching = 0;
  vector<int> indices(1);
  vector<float> sq_distances(1);
  const float kSquaredTouchingDistance =
      distance_threshold * distance_threshold;

  const bool kDebugDistances = context->kDebug && false;
  int histogram[5];
  if (kDebugDistances) {
    histogram[0] = 0;
    histogram[1] = 0;
    histogram[2] = 0;
    histogram[3] = 0;
    histogram[4] = 0;
  }
  for (size_t index_i = 0; index_i < hand_indices->size(); ++index_i) {
    int index = (*hand_indices)[index_i];
    object_tree->nearestKSearch(both_hands_cloud->points[index], 1, indices,
                                sq_distances);
    float sq_distance = sq_distances[0];
    if (sq_distance < kSquaredTouchingDistance) {
      num_touching += 1;
    }

    if (kDebugDistances) {
      float distance = sqrt(sq_distance);
      if (distance < 0.002) {
        histogram[0] += 1;
      }
      if (distance < 0.005) {
        histogram[1] += 1;
      }
      if (distance < 0.008) {
        histogram[2] += 1;
      }
      if (distance < 0.01) {
        histogram[3] += 1;
      }
      if (distance < 0.012) {
        histogram[4] += 1;
      }
    }
  }
  if (kDebugDistances) {
    ROS_INFO("%d %d %d %d %d", histogram[0], histogram[1], histogram[2],
             histogram[3], histogram[4]);
  }
  return num_touching;
}

bool ContactDetection::IsObjectMoving(const msgs::ObjectState& object,
                                      ContactDetectionContext* context) const {
  msgs::ObjectState prev_obj;
  if (!context->GetPreviousObject(object.name, &prev_obj)) {
    return false;
  }

  ros::Duration dt = context->GetCurrentTime() - context->GetPreviousTime();

  // TODO: we only take linear movement into account, but not angular motion
  // Angular motion may be hard to track.
  double linear_distance = LinearDistance(prev_obj.pose, object.pose);
  // double linear_speed = linear_distance / dt.toSec();
  // ROS_INFO("%s: moved %f in %f seconds (%f m/s)", object.name.c_str(),
  //         linear_distance, dt.toSec(), linear_speed);
  return linear_distance >= context->kMovingObjectDistance;
}

void ContactDetection::PublishWristPoses(const geometry_msgs::Pose& left,
                                         const geometry_msgs::Pose& right,
                                         const string& frame_id) {
  visualization_msgs::Marker left_marker;
  left_marker.header.frame_id = frame_id;
  left_marker.ns = "left_wrist";
  left_marker.id = 0;
  left_marker.type = visualization_msgs::Marker::ARROW;
  left_marker.pose = left;
  left_marker.scale.x = 0.1;
  left_marker.scale.y = 0.01;
  left_marker.scale.z = 0.01;
  left_marker.color.b = 1;
  left_marker.color.a = 1;
  viz_.publish(left_marker);

  visualization_msgs::Marker right_marker = left_marker;
  right_marker.ns = "right_wrist";
  right_marker.pose = right;
  viz_.publish(right_marker);
}

ContactDetectionContext::ContactDetectionContext(
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

bool ContactDetectionContext::LoadParams() {
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

const geometry_msgs::Pose& ContactDetectionContext::GetLeftWristPose() {
  GetWristPoses();
  return left_wrist_pose_;
}

const geometry_msgs::Pose& ContactDetectionContext::GetRightWristPose() {
  GetWristPoses();
  return right_wrist_pose_;
}

const vector<msgs::ObjectState>& ContactDetectionContext::GetPrevObjects()
    const {
  return prev_state_.object_states;
}

const vector<msgs::ObjectState>& ContactDetectionContext::GetCurrentObjects()
    const {
  return current_state_.object_states;
}

PointCloudP::Ptr ContactDetectionContext::GetObjectModel(const string& name) {
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

PointCloudP::Ptr ContactDetectionContext::GetObjectCloud(const string& name) {
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

KdTreeP::Ptr ContactDetectionContext::GetObjectTree(const string& name) {
  if (object_trees_.find(name) == object_trees_.end()) {
    KdTreeP::Ptr tree(new pcl::KdTreeFLANN<PointP>);
    tree->setInputCloud(GetObjectCloud(name));
    object_trees_[name] = tree;
  }
  return object_trees_[name];
}

bool ContactDetectionContext::GetCurrentObject(
    const std::string& name, task_perception_msgs::ObjectState* object) {
  IndexObjects();
  if (current_objects_.find(name) == current_objects_.end()) {
    return false;
  }
  *object = current_objects_[name];
  return true;
}

bool ContactDetectionContext::GetPreviousObject(
    const std::string& name, task_perception_msgs::ObjectState* prev_obj) {
  IndexObjects();
  if (prev_objects_.find(name) == prev_objects_.end()) {
    return false;
  }
  *prev_obj = prev_objects_[name];
  return true;
}

void ContactDetectionContext::GetWristPoses() {
  if (!have_wrist_poses_) {
    ss_msgs::GetSkeletonStateRequest state_req;
    ss_msgs::GetSkeletonStateResponse state_res;
    skel_services_.get_state.call(state_req, state_res);
    left_wrist_pose_ = state_res.left_wrist;
    right_wrist_pose_ = state_res.right_wrist;
    have_wrist_poses_ = true;
  }
}

void ContactDetectionContext::IndexObjects() {
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

PointCloudP::Ptr ContactDetectionContext::LoadModel(const string& mesh_path) {
  PointCloudP::Ptr object_model(new PointCloudP);
  pcl::io::loadPCDFile(mesh_path, *object_model);
  ROS_INFO("Loaded mesh %s with %ld points", mesh_path.c_str(),
           object_model->size());
  return object_model;
}

ros::Time ContactDetectionContext::GetPreviousTime() {
  return prev_state_.stamp;
}

ros::Time ContactDetectionContext::GetCurrentTime() {
  return current_state_.stamp;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ContactDetectionContext::BothHandsCloud() {
  if (!both_hands_cloud_) {
    ComputeHandClouds();
  }
  return both_hands_cloud_;
}

pcl::IndicesPtr ContactDetectionContext::LeftHandIndices() {
  if (!left_hand_indices_) {
    ComputeHandClouds();
  }
  return left_hand_indices_;
}

pcl::IndicesPtr ContactDetectionContext::RightHandIndices() {
  if (!right_hand_indices_) {
    ComputeHandClouds();
  }
  return right_hand_indices_;
}

pcl::KdTree<pcl::PointXYZ>::Ptr ContactDetectionContext::LeftHandTree() {
  if (!left_hand_tree_) {
    left_hand_tree_.reset(new pcl::KdTreeFLANN<PointP>);
    if (left_hand_indices_->size() > 0) {
      left_hand_tree_->setInputCloud(both_hands_cloud_, left_hand_indices_);
    }
  }
  return left_hand_tree_;
}

pcl::KdTree<pcl::PointXYZ>::Ptr ContactDetectionContext::RightHandTree() {
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
void ContactDetectionContext::ComputeHandClouds() {
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
void ContactDetectionContext::ComputeLeftRightHands() {
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
