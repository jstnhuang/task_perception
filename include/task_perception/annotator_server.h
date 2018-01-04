#ifndef _PBI_ANNOTATOR_SERVER_H_
#define _PBI_ANNOTATOR_SERVER_H_

#include <map>
#include <memory>
#include <string>

#include "boost/shared_ptr.hpp"
//#include "dbot/object_resource_identifier.h"
//#include "dbot_ros/util/interactive_marker_initializer.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/NerfJointStates.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"

//#include "task_perception/track.h"
#include "task_perception/video_scrubber.h"

namespace pbi {
class AnnotatorServer {
 public:
  AnnotatorServer(const ros::Publisher& camera_info_pub,
                  const ros::Publisher& color_pub,
                  const ros::Publisher& depth_pub,
                  const ros::Publisher& state_pub);
  void Start();
  void HandleEvent(const task_perception_msgs::AnnotatorEvent& event);

 private:
  void HandleOpen(const std::string& bag_path);
  void HandleStep();
  void HandleAddObject(const std::string& mesh_name);

  void ProcessCurrentStep();

  // Loop that continuously publishes the RGBD image. This is needed for the
  // depthcloud_encoder node.
  void Loop(const ros::TimerEvent& event);
  void PublishState();

  ros::Publisher camera_info_pub_;
  ros::Publisher color_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher state_pub_;

  ros::NodeHandle nh_;
  ros::Timer timer_;

  // Bag file state
  boost::shared_ptr<rosbag::Bag> bag_;
  std::string color_topic_;
  std::string depth_topic_;
  sensor_msgs::CameraInfo camera_info_;

  task_perception_msgs::AnnotatorState state_;

  VideoScrubber color_scrubber_;
  VideoScrubber depth_scrubber_;
  sensor_msgs::Image current_color_image_;
  sensor_msgs::Image current_depth_image_;

  // Skeleton tracker
  ros::ServiceClient reset_skeleton;
  ros::ServiceClient advance_skeleton;

  // Object tracking
  // std::map<std::string, Track> tracks_;
};
}  // namespace pbi

#endif  // _PBI_ANNOTATOR_SERVER_H_
