#include "task_perception/annotator_server.h"

#include "boost/shared_ptr.hpp"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"

#include "task_perception/names.h"

namespace msgs = task_perception_msgs;
using sensor_msgs::Image;

namespace pbi {
AnnotatorServer::AnnotatorServer(const ros::Publisher& color_pub,
                                 const ros::Publisher& state_pub,
                                 const VideoScrubber& scrubber)
    : color_pub_(color_pub),
      state_pub_(state_pub),
      scrubber_(scrubber),
      nh_(),
      timer_(nh_.createTimer(ros::Duration(1 / 15.0), &AnnotatorServer::Loop,
                             this)),
      current_image_(),
      bag_(),
      state_() {}

void AnnotatorServer::Start() {
  timer_.start();
  state_pub_.publish(state_);
}

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
  if (!current_image_.header.stamp.isZero()) {
    color_pub_.publish(current_image_);
  }
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
  state_.bag_path = bag_path;
  if (color_images.size() == 0) {
    ROS_ERROR("No images in bag!");
    return;
  }
  state_.bag_length =
      color_images.back().header.stamp - color_images[0].header.stamp;
  state_pub_.publish(state_);
}
}  // namespace pbi
