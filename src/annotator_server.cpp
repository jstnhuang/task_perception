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
#include "skin_segmentation_msgs/AdvanceSkeleton.h"
#include "skin_segmentation_msgs/ResetSkeletonTracker.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"

#include "task_perception/bag_utils.h"
#include "task_perception/names.h"
//#include "task_perception/particle_tracker_builder.h"
//#include "task_perception/track.h"
#include "task_perception/video_scrubber.h"

namespace msgs = task_perception_msgs;
namespace ss_msgs = skin_segmentation_msgs;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;
typedef message_filters::sync_policies::ExactTime<Image, Image> MyPolicy;
// typedef dbot::ObjectTrackerRos<dbot::ParticleTracker> ParticleTrackerRos;
// typedef std::shared_ptr<ParticleTrackerRos> ParticleTrackerRosPtr;

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
      reset_skeleton(nh_.serviceClient<ss_msgs::ResetSkeletonTracker>(
          "reset_skeleton_tracker")),
      advance_skeleton(
          nh_.serviceClient<ss_msgs::AdvanceSkeleton>("advance_skeleton")) {
  while (!reset_skeleton.waitForExistence(ros::Duration(2.0)) ||
         !advance_skeleton.waitForExistence(ros::Duration(2.0))) {
    ROS_WARN("Waiting for skeleton tracking service");
  }
}

void AnnotatorServer::Start() { state_pub_.publish(state_); }

void AnnotatorServer::HandleEvent(
    const task_perception_msgs::AnnotatorEvent& event) {
  try {
    if (event.type == msgs::AnnotatorEvent::OPEN_BAG) {
      HandleOpen(event.bag_path);
    } else if (event.type == msgs::AnnotatorEvent::STEP) {
      HandleStep();
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

  // Load images
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

  // Initialize skeleton tracker
  ss_msgs::ResetSkeletonTrackerRequest reset_req;
  ss_msgs::ResetSkeletonTrackerResponse reset_res;
  reset_req.rgb_topic = color_topic_;
  reset_req.depth_topic = depth_topic_;
  reset_req.camera_info = camera_info_;
  reset_skeleton.call(reset_req, reset_res);

  ROS_INFO("Opened bag: %s with %d frames", bag_path.c_str(), num_frames);
  state_.bag_path = bag_path;
  state_.frame_count = num_frames;
  state_.current_frame = 0;

  ProcessCurrentStep();
}

void AnnotatorServer::HandleStep() {
  state_.current_frame += 1;
  if (state_.current_frame >= state_.frame_count) {
    ROS_INFO("Reached end of bag file.");
    return;
  }
  ProcessCurrentStep();
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

void AnnotatorServer::ProcessCurrentStep() {
  if (state_.current_frame < 0 || state_.current_frame >= state_.frame_count) {
    ROS_ERROR("Invalid step %d (%d).", state_.current_frame,
              state_.frame_count);
    return;
  }

  color_scrubber_.View(state_.current_frame, &current_color_image_);
  depth_scrubber_.View(state_.current_frame, &current_depth_image_);

  ss_msgs::AdvanceSkeletonRequest advance_req;
  ss_msgs::AdvanceSkeletonResponse advance_res;
  advance_req.rgb = current_color_image_;
  advance_req.depth = current_depth_image_;
  advance_skeleton.call(advance_req, advance_res);

  PublishState();
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

  // color_scrubber_.View(state_.current_frame, &current_color_image_);
  // depth_scrubber_.View(state_.current_frame, &current_depth_image_);

  // TODO: skeleton tracker manages its own state and visualization for now,
  // maybe move into here
}
}  // namespace pbi
