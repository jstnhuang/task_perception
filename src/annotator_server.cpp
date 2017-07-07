#include "task_perception/annotator_server.h"

#include "boost/shared_ptr.hpp"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/AnnotatorEvent.h"

#include "task_perception/names.h"

namespace msgs = task_perception_msgs;
using sensor_msgs::Image;

namespace pbi {
AnnotatorServer::AnnotatorServer(const ros::Publisher& color_pub,
                                 const VideoScrubber& scrubber)
    : color_pub_(color_pub),
      scrubber_(scrubber),
      nh_(),
      timer_(nh_.createTimer(ros::Duration(1 / 15.0), &AnnotatorServer::Loop,
                             this)),
      current_image_(),
      bag_() {}

void AnnotatorServer::Start() { timer_.start(); }

void AnnotatorServer::HandleEvent(
    const task_perception_msgs::AnnotatorEvent& event) {
  try {
    if (event.type == msgs::AnnotatorEvent::OPEN_BAG) {
      HandleOpen(event.bag_path);
    } else if (event.type == msgs::AnnotatorEvent::VIEW_TIME) {
      scrubber_.View(event.time, &current_image_);
    } else {
      ROS_ERROR("Unknown event type: \"%s\"", event.type.c_str());
    }
  } catch (const rosbag::BagException& ex) {
    ROS_ERROR("rosbag exception: %s", ex.what());
  }
}

void AnnotatorServer::Loop(const ros::TimerEvent& event) {
  color_pub_.publish(current_image_);
}

void AnnotatorServer::HandleOpen(const std::string& bag_path) {
  if (bag_) {
    bag_->close();
  }
  bag_.reset(new rosbag::Bag);
  bag_->open(bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(bag::kColorTopic);
  rosbag::View view(*bag_, rosbag::TopicQuery(topics));
  std::vector<Image> color_images;
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    const Image::ConstPtr& image = it->instantiate<Image>();
    if (image != NULL) {
      color_images.push_back(*image);
    }
  }
  scrubber_.set_images(color_images);
  ROS_INFO("Opened bag: %s", bag_path.c_str());
}
}  // namespace pbi
