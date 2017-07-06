#ifndef _PBI_RECORD_VIDEO_ACTION_SERVER_H_
#define _PBI_RECORD_VIDEO_ACTION_SERVER_H_

#include <string>

#include "actionlib/server/simple_action_server.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/RecordVideoAction.h"
#include "tf/transform_listener.h"

#include "boost/shared_ptr.hpp"

namespace pbi {
// Action for recording a task demonstration video.
// This records the following:
// - color_in: A sensor_msgs/Image topic, the RGB data
// - depth_in: A sensor_msgs/Image topic, the depth data
//
// This is only recorded once:
// - camera_info: A sensor_msgs/CameraInfo topic
//
// These topics will need to be remapped to the actual color, depth, and camera
// info topics.
//
// This action also records the transformation that describes the frame_id
// specified in the camera info in the base_link frame as a
// geometry_msgs/Transform to the camera_transform topic. This is only computed
// and saved once (i.e., we assume that the camera is stationary during the
// video).
const static char kColorTopic[] = "color_in";
const static char kDepthTopic[] = "depth_in";
const static char kCameraInfoTopic[] = "camera_info";
const static char kCameraTransformTopic[] = "camera_transform";

class RecordVideoActionServer {
 public:
  RecordVideoActionServer();
  void Start();

 private:
  void Execute(const task_perception_msgs::RecordVideoGoalConstPtr& goal);
  void ColorCallback(const sensor_msgs::Image& image);
  void DepthCallback(const sensor_msgs::Image& image);

  void PublishFeedback(const ros::TimerEvent& event);
  void Finish(const std::string& error);

  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber color_sub_;
  ros::Subscriber depth_sub_;
  actionlib::SimpleActionServer<task_perception_msgs::RecordVideoAction> as_;

  boost::shared_ptr<rosbag::Bag> current_bag_;
};
}  // namespace pbi

#endif  // _PBI_RECORD_VIDEO_ACTION_SERVER_H_
