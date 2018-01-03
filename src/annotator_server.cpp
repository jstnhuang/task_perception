#include "task_perception/annotator_server.h"

#include <map>
#include <memory>

#include "boost/shared_ptr.hpp"
//#include "dbot/object_resource_identifier.h"
//#include "dbot_ros/util/interactive_marker_initializer.h"
#include "geometry_msgs/Transform.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"
#include "tf/transform_broadcaster.h"

#include "task_perception/bag_utils.h"
#include "task_perception/names.h"
#include "task_perception/particle_tracker_builder.h"
#include "task_perception/track.h"
#include "task_perception/video_scrubber.h"

namespace msgs = task_perception_msgs;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;
typedef dbot::ObjectTrackerRos<dbot::ParticleTracker> ParticleTrackerRos;
typedef std::shared_ptr<ParticleTrackerRos> ParticleTrackerRosPtr;
typedef message_filters::sync_policies::ExactTime<Image, Image> MyPolicy;

namespace pbi {
AnnotatorServer::AnnotatorServer(const ros::Publisher& camera_info_pub,
                                 const ros::Publisher& color_pub,
                                 const ros::Publisher& depth_pub,
                                 const ros::Publisher& state_pub)
    : camera_info_pub_(camera_info_pub),
      color_pub_(color_pub),
      depth_pub_(depth_pub),
      state_pub_(state_pub),
      nh_(),
      timer_(nh_.createTimer(ros::Duration(1 / 15.0), &AnnotatorServer::Loop,
                             this)),
      bag_(),
      color_topic_(""),
      depth_topic_(""),
      camera_info_(),
      state_(),
      color_scrubber_(),
      depth_scrubber_(),
      current_color_image_(),
      current_depth_image_(),
      tracks_() {}

void AnnotatorServer::Start() { state_pub_.publish(state_); }

void AnnotatorServer::HandleEvent(
    const task_perception_msgs::AnnotatorEvent& event) {
  try {
    if (event.type == msgs::AnnotatorEvent::OPEN_BAG) {
      HandleOpen(event.bag_path);
    } else if (event.type == msgs::AnnotatorEvent::VIEW_FRAME) {
      HandleViewFrame(event.frame_number);
    } else if (event.type == msgs::AnnotatorEvent::ADD_OBJECT) {
      HandleAddObject(event.mesh_name);
    } else {
      ROS_ERROR("Unknown event type: \"%s\"", event.type.c_str());
    }
  } catch (const rosbag::BagException& ex) {
    ROS_ERROR("rosbag exception: %s", ex.what());
    bag_.reset();
  }
}

void AnnotatorServer::HandleOpen(const std::string& bag_path) {
  if (bag_) {
    bag_->close();
  }
  bag_.reset(new rosbag::Bag);
  bag_->open(bag_path, rosbag::bagmode::Read);

  GetImageTopics(*bag_, &color_topic_, &depth_topic_);
  GetCameraInfo(*bag_, &camera_info_);
  int num_frames = GetNumMessagesOnTopic(*bag_, depth_topic_);

  std::vector<std::string> topics(2);
  topics[0] = color_topic_;
  topics[1] = depth_topic_;
  rosbag::View view(*bag_, rosbag::TopicQuery(topics));
  std::vector<Image> color_images;
  std::vector<Image> depth_images;
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    if (it->getTopic() == color_topic_) {
      color_images.push_back(*it->instantiate<Image>());
    } else if (it->getTopic() == depth_topic_) {
      depth_images.push_back(*it->instantiate<Image>());
    }
  }
  color_scrubber_.set_images(color_images);
  depth_scrubber_.set_images(depth_images);

  ROS_INFO("Opened bag: %s with %d frames", bag_path.c_str(), num_frames);
  state_.bag_path = bag_path;
  state_.frame_count = num_frames;
  state_.current_frame = 0;

  PublishState();
}

void AnnotatorServer::HandleViewFrame(const int frame_index) {
  int frame_idx = frame_index;
  if (frame_index < 0) {
    ROS_WARN("Out of bounds frame index: %d (%d)", frame_index,
             state_.frame_count);
    frame_idx = 0;
  } else if (frame_index >= state_.frame_count) {
    ROS_WARN("Out of bounds frame index: %d (%d)", frame_index,
             state_.frame_count);
    frame_idx = state_.frame_count - 1;
  }
  state_.current_frame = frame_index;
  PublishState();
}

void AnnotatorServer::HandleAddObject(const std::string& mesh_name) {
  // TODO: we assume object exists since the beginning of the demonstration
  // i.e., depth_frame is 0.
  // if (tracks_.find(mesh_name) == tracks_.end()) {
  //  dbot::ObjectResourceIdentifier ori;
  //  BuildOri(nh_, mesh_name, &ori);
  //  Track track(nh_, ori, &depth_scrubber_);
  //  tracks_.insert(std::pair<std::string, Track>(mesh_name, track));

  //  ROS_INFO("Created tracker for %s", mesh_name.c_str());
  //}
}

void AnnotatorServer::Loop(const ros::TimerEvent& event) {
  ros::Time now = ros::Time::now();
  CameraInfo cam_info = camera_info_;
  cam_info.header.stamp = now;
  camera_info_pub_.publish(cam_info);

  if (!current_color_image_.header.stamp.isZero()) {
    current_color_image_.header.stamp = now;
    color_pub_.publish(current_color_image_);
  }
  if (!current_depth_image_.header.stamp.isZero()) {
    current_depth_image_.header.stamp = now;
    depth_pub_.publish(current_depth_image_);
  }
}

void AnnotatorServer::PublishState() {
  state_pub_.publish(state_);

  color_scrubber_.View(state_.current_frame, &current_color_image_);
  depth_scrubber_.View(state_.current_frame, &current_depth_image_);
}
}  // namespace pbi
