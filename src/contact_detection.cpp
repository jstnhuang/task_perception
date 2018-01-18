#include "task_perception/contact_detection.h"

#include <string>

#include "pcl/common/transforms.h"
#include "pcl/io/obj_io.h"
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
      package_dir_("") {
  package_dir_ = ros::package::getPath("object_meshes") + "/object_models/";
  ROS_INFO("Mesh package dir: %s", package_dir_.c_str());
}

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

  // Check if hand is close to an object
  for (const auto& object : current_state.object_states) {
    std::string mesh_path = package_dir_ + object.mesh_name;
    PointCloudP::Ptr object_model(new PointCloudP);
    PointCloudP::Ptr object_cloud(new PointCloudP);
    pcl::io::loadOBJFile(mesh_path, *object_model);
    object_model->header.frame_id = camera_info.header.frame_id;
    ROS_INFO("Loaded object model %s with %ld points", object.mesh_name.c_str(),
             object_model->size());
    PublishPointCloud(obj_viz_, *object_model);
    // pcl::transformPointCloud(object_model, object_cloud, transform);
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
}  // namespace pbi
