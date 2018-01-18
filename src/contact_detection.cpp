#include "task_perception/contact_detection.h"

#include <map>
#include <string>

#include "Eigen/Dense"

#include "absl/strings/str_cat.h"
#include "absl/strings/strip.h"
#include "eigen_conversions/eigen_msg.h"
#include "image_geometry/pinhole_camera_model.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
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
#include "task_perception/skeleton_services.h"

namespace msgs = task_perception_msgs;
namespace ss_msgs = skin_segmentation_msgs;

namespace pbi {
ContactDetection::ContactDetection(const SkeletonServices& skel_services,
                                   const ros::ServiceClient& predict_hands)
    : skel_services_(skel_services),
      predict_hands_(predict_hands),
      debug_(false),
      nh_(),
      viz_(nh_.advertise<visualization_msgs::Marker>(
          "contact_detection/markers", 10)),
      obj_viz_(nh_.advertise<sensor_msgs::PointCloud2>(
          "contact_detection/object_clouds", 1, true)),
      package_dir_(ros::package::getPath("object_meshes") + "/object_models/"),
      model_cache_() {}

void ContactDetection::Predict(
    const task_perception_msgs::DemoState& current_state,
    const task_perception_msgs::DemoState& prev_state,
    const sensor_msgs::Image& color_image,
    const sensor_msgs::Image& depth_image,
    const sensor_msgs::CameraInfo& camera_info) {
  ros::param::param("contact_detection/debug", debug_, false);

  // Find wrist poses
  ss_msgs::GetSkeletonStateRequest state_req;
  ss_msgs::GetSkeletonStateResponse state_res;
  skel_services_.get_state.call(state_req, state_res);
  const geometry_msgs::Pose& left_wrist = state_res.left_wrist;
  const geometry_msgs::Pose& right_wrist = state_res.right_wrist;

  if (debug_) {
    PublishWristPoses(left_wrist, right_wrist, camera_info.header.frame_id);
  }

  // Find hand pixels
  ss_msgs::PredictHandsRequest req;
  req.rgb = color_image;
  req.depth_registered = depth_image;
  ss_msgs::PredictHandsResponse res;
  predict_hands_.call(req, res);
  const sensor_msgs::Image& hands = res.prediction;

  // Create point cloud for hands
  PointCloudP::Ptr hand_cloud = HandPointCloud(hands, depth_image, camera_info);
  PublishPointCloud(obj_viz_, *hand_cloud);

  // Check if hand is close to an object
  for (const auto& object : current_state.object_states) {
    PointCloudP::Ptr object_model = LoadModel(object.mesh_name);
    object_model->header.frame_id = camera_info.header.frame_id;
    Eigen::Affine3d object_transform;
    tf::poseMsgToEigen(object.object_pose, object_transform);
    PointCloudP::Ptr object_cloud(new PointCloudP);
    pcl::transformPointCloud(*object_model, *object_cloud, object_transform);
  }
}

void ContactDetection::PublishWristPoses(const geometry_msgs::Pose& left,
                                         const geometry_msgs::Pose& right,
                                         const std::string& frame_id) {
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

PointCloudP::Ptr ContactDetection::LoadModel(const std::string& mesh_name_obj) {
  if (model_cache_.find(mesh_name_obj) == model_cache_.end()) {
    absl::string_view pcd_file(mesh_name_obj);
    absl::ConsumeSuffix(&pcd_file, ".obj");
    std::string mesh_path = absl::StrCat(package_dir_, pcd_file, ".pcd");
    PointCloudP::Ptr object_model(new PointCloudP);
    PointCloudP::Ptr object_cloud(new PointCloudP);
    pcl::io::loadPCDFile(mesh_path, *object_model);
    model_cache_[mesh_name_obj] = object_model;
    ROS_INFO("Loaded mesh %s with %ld points", mesh_name_obj.c_str(),
             object_model->size());
  }
  return model_cache_[mesh_name_obj];
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ContactDetection::HandPointCloud(
    const sensor_msgs::Image& hands, const sensor_msgs::Image& depth,
    const sensor_msgs::CameraInfo& camera_info) {
  PointCloudP::Ptr hand_cloud(new PointCloudP);
  if (hands.encoding != "mono8") {
    ROS_ERROR("Unsupported hand prediction format: %s", hands.encoding.c_str());
    return hand_cloud;
  }

  const uint16_t* depth_16;
  const float* depth_float;
  bool is_float = depth.encoding == "32FC1";
  if (is_float) {
    depth_float = reinterpret_cast<const float*>(depth.data.data());
  } else if (depth.encoding == "16UC1") {
    depth_16 = reinterpret_cast<const uint16_t*>(depth.data.data());
  } else {
    ROS_ERROR("Unsupported depth image type: %s", depth.encoding.c_str());
    return hand_cloud;
  }

  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);
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
        hand_cloud->push_back(pt);
      }
    }
  }
  hand_cloud->header.frame_id = camera_info.header.frame_id;
  return hand_cloud;
}
}  // namespace pbi
