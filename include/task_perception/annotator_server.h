#ifndef _PBI_ANNOTATOR_SERVER_H_
#define _PBI_ANNOTATOR_SERVER_H_

#include <string>

#include "boost/shared_ptr.hpp"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/Image.h"
#include "task_perception/video_scrubber.h"
#include "task_perception_msgs/AnnotatorEvent.h"

namespace pbi {
class AnnotatorServer {
 public:
  AnnotatorServer(const ros::Publisher& color_pub,
                  const VideoScrubber& scrubber);
  void Start();
  void HandleEvent(const task_perception_msgs::AnnotatorEvent& event);

 private:
  void Loop(const ros::TimerEvent& event);
  void HandleOpen(const std::string& bag_path);
  ros::Publisher color_pub_;
  VideoScrubber scrubber_;

  ros::NodeHandle nh_;
  ros::Timer timer_;
  sensor_msgs::Image current_image_;
  boost::shared_ptr<rosbag::Bag> bag_;
};
}  // namespace pbi

#endif  // _PBI_ANNOTATOR_SERVER_H_
