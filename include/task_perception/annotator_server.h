#ifndef _PBI_ANNOTATOR_SERVER_H_
#define _PBI_ANNOTATOR_SERVER_H_

#include <map>
#include <memory>
#include <string>

#include "boost/shared_ptr.hpp"
#include "dbot/object_resource_identifier.h"
#include "dbot_ros/util/interactive_marker_initializer.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"
#include "tf/transform_broadcaster.h"

#include "task_perception/track.h"
#include "task_perception/video_scrubber.h"

namespace pbi {
class AnnotatorServer {
 public:
  AnnotatorServer(const ros::Publisher& camera_info_pub,
                  const ros::Publisher& color_pub,
                  const ros::Publisher& depth_pub,
                  const ros::Publisher& state_pub,
                  const tf::TransformBroadcaster& tf_broadcaster,
                  const std::string& camera_frame);
  void Start();
  void HandleEvent(const task_perception_msgs::AnnotatorEvent& event);

 private:
  void Loop(const ros::TimerEvent& event);
  void HandleOpen(const std::string& bag_path);
  void HandleViewDepthFrame(int frame_index);
  void HandleAddObject(const std::string& mesh_name);
  ros::Publisher camera_info_pub_;
  ros::Publisher color_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher state_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  opi::InteractiveMarkerInitializer im_init_;

  ros::NodeHandle nh_;
  ros::Timer timer_;
  VideoScrubber color_scrubber_;
  VideoScrubber depth_scrubber_;
  sensor_msgs::CameraInfo current_camera_info_;
  sensor_msgs::Image current_color_image_;
  sensor_msgs::Image current_depth_image_;
  geometry_msgs::Transform camera_frame_;
  boost::shared_ptr<rosbag::Bag> bag_;
  task_perception_msgs::AnnotatorState state_;

  std::map<std::string, Track> tracks_;
};
}  // namespace pbi

#endif  // _PBI_ANNOTATOR_SERVER_H_
