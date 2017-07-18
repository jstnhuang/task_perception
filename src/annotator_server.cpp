#include "task_perception/annotator_server.h"

#include <map>
#include <memory>

#include "boost/shared_ptr.hpp"
#include "dbot/object_resource_identifier.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"
#include "tf/transform_broadcaster.h"

#include "task_perception/names.h"
#include "task_perception/particle_tracker_builder.h"
#include "task_perception/track.h"
#include "task_perception/video_scrubber.h"

namespace msgs = task_perception_msgs;
using sensor_msgs::Image;
typedef dbot::ObjectTrackerRos<dbot::ParticleTracker> ParticleTrackerRos;
typedef std::shared_ptr<ParticleTrackerRos> ParticleTrackerRosPtr;

namespace pbi {
AnnotatorServer::AnnotatorServer(const ros::Publisher& camera_info_pub,
                                 const ros::Publisher& color_pub,
                                 const ros::Publisher& depth_pub,
                                 const ros::Publisher& state_pub,
                                 const tf::TransformBroadcaster& tf_broadcaster)
    : camera_info_pub_(camera_info_pub),
      color_pub_(color_pub),
      depth_pub_(depth_pub),
      state_pub_(state_pub),
      tf_broadcaster_(tf_broadcaster),
      nh_(),
      timer_(nh_.createTimer(ros::Duration(1 / 15.0), &AnnotatorServer::Loop,
                             this)),
      color_scrubber_(),
      depth_scrubber_(),
      current_color_image_(),
      current_depth_image_(),
      bag_(),
      state_(),
      tracks_() {}

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
      color_scrubber_.View(event.time, &current_color_image_);
      depth_scrubber_.View(event.time, &current_depth_image_);
    } else if (event.type == msgs::AnnotatorEvent::VIEW_DEPTH_FRAME) {
      HandleViewDepthFrame(event.depth_frame);
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

void AnnotatorServer::Loop(const ros::TimerEvent& event) {
  if (!current_color_image_.header.stamp.isZero()) {
    color_pub_.publish(current_color_image_);
  }
  if (!current_depth_image_.header.stamp.isZero()) {
    depth_pub_.publish(current_depth_image_);
  }
  if (bag_) {
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = kBaseFrame;
    ts.child_frame_id = kCameraFrame;
    ts.transform = camera_frame_;
    tf_broadcaster_.sendTransform(ts);

    camera_info_pub_.publish(current_camera_info_);
  }
}

void AnnotatorServer::HandleOpen(const std::string& bag_path) {
  if (bag_) {
    bag_->close();
  }
  bag_.reset(new rosbag::Bag);
  bag_->open(bag_path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(bag::kCameraInfoTopic);
  topics.push_back(bag::kColorTopic);
  topics.push_back(bag::kDepthTopic);
  topics.push_back(bag::kCameraTransformTopic);
  rosbag::View view(*bag_, rosbag::TopicQuery(topics));
  std::vector<Image> color_images;
  std::vector<Image> depth_images;
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    if (it->getTopic() == bag::kColorTopic) {
      const Image::ConstPtr& image = it->instantiate<Image>();
      if (image != NULL) {
        color_images.push_back(*image);
      }
    } else if (it->getTopic() == bag::kDepthTopic) {
      const Image::ConstPtr& image = it->instantiate<Image>();
      if (image != NULL) {
        depth_images.push_back(*image);
      }
    } else if (it->getTopic() == bag::kCameraTransformTopic) {
      const geometry_msgs::Transform::ConstPtr& camera_frame =
          it->instantiate<geometry_msgs::Transform>();
      if (camera_frame != NULL) {
        camera_frame_ = *camera_frame;
      }
    } else if (it->getTopic() == bag::kCameraInfoTopic) {
      const sensor_msgs::CameraInfo::ConstPtr& camera_info =
          it->instantiate<sensor_msgs::CameraInfo>();
      if (camera_info != NULL) {
        current_camera_info_ = *camera_info;
      }
    }
  }
  color_scrubber_.set_images(color_images);
  depth_scrubber_.set_images(depth_images);
  ROS_INFO("Opened bag: %s", bag_path.c_str());
  state_.bag_path = bag_path;
  if (color_images.size() == 0 || depth_images.size() == 0) {
    ROS_ERROR("No images in bag!");
    return;
  }
  state_.bag_length =
      color_images.back().header.stamp - color_images[0].header.stamp;
  state_pub_.publish(state_);
}

void AnnotatorServer::HandleViewDepthFrame(int frame_index) {
  for (auto& kv : tracks_) {
    Track& track = kv.second;
    if (frame_index < track.current_frame_index()) {
      track.Reset();
    }
    while (track.current_frame_index() <= frame_index) {
      track.Step();
    }
  }
}

void AnnotatorServer::HandleAddObject(const std::string& mesh_name) {
  // TODO: we assume object exists since the beginning of the demonstration
  // i.e., depth_frame is 0.
  if (tracks_.find(mesh_name) == tracks_.end()) {
    dbot::ObjectResourceIdentifier ori;
    BuildOri(nh_, mesh_name, &ori);
    Track track(nh_, ori, &depth_scrubber_);
    tracks_.insert(std::pair<std::string, Track>(mesh_name, track));

    ROS_INFO("Created tracker for %s", mesh_name.c_str());
  }
}
}  // namespace pbi
