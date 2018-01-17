#include "task_perception/contact_detection.h"

#include <string>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/GetSkeletonState.h"
#include "skin_segmentation_msgs/PredictHands.h"
#include "task_perception_msgs/DemoState.h"
#include "visualization_msgs/Marker.h"

#include "task_perception/skeleton_services.h"

namespace msgs = task_perception_msgs;
namespace ss_msgs = skin_segmentation_msgs;

namespace pbi {
ContactDetection::ContactDetection(const SkeletonServices& skel_services,
                                   const ros::ServiceClient& predict_hands)
    : skel_services_(skel_services),
      predict_hands_(predict_hands),
      nh_(),
      viz_(nh_.advertise<visualization_msgs::Marker>(
          "contact_detection/markers", 10)),
      hand_viz_(nh_.advertise<sensor_msgs::Image>("segmented_hand", 1)) {}

void ContactDetection::Predict(
    const task_perception_msgs::DemoState& current_state,
    const task_perception_msgs::DemoState& prev_state,
    const sensor_msgs::Image& color_image,
    const sensor_msgs::Image& depth_image,
    const sensor_msgs::CameraInfo& camera_info) {
  // Find wrist poses
  ss_msgs::GetSkeletonStateRequest state_req;
  ss_msgs::GetSkeletonStateResponse state_res;
  skel_services_.get_state.call(state_req, state_res);
  const geometry_msgs::Pose& left_wrist = state_res.left_wrist;
  const geometry_msgs::Pose& right_wrist = state_res.right_wrist;

  PublishWristPoses(left_wrist, right_wrist, camera_info.header.frame_id);

  // Find hand pixels
  ss_msgs::PredictHandsRequest req;
  req.rgb = color_image;
  req.depth_registered = depth_image;
  ss_msgs::PredictHandsResponse res;
  predict_hands_.call(req, res);
  const sensor_msgs::Image& hands = res.prediction;
  hand_viz_.publish(hands);
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
