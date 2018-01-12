#include "task_perception/demo_runtime.h"

#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/AdvanceSkeleton.h"
#include "skin_segmentation_msgs/ResetSkeletonTracker.h"
#include "task_perception_msgs/Demonstration.h"

#include "task_perception/default_skeleton.h"
#include "task_perception/demo_model.h"
#include "task_perception/demo_visualizer.h"
#include "task_perception/object_tracker.h"
#include "task_perception/skeleton_services.h"
#include "task_perception/video_scrubber.h"

namespace ss_msgs = skin_segmentation_msgs;
namespace msgs = task_perception_msgs;

namespace pbi {
DemoRuntime::DemoRuntime(const DemoVisualizer& viz,
                         const SkeletonServices& skel_services)
    : viz_(viz),
      skel_services_(skel_services),
      demo_model_(),
      nh_(),
      color_scrubber_(),
      depth_scrubber_(),
      current_color_image_(),
      current_depth_image_(),
      camera_info_(),
      object_trackers_(),
      frame_number_(0),
      num_frames_(0),
      states_() {}

void DemoRuntime::LoadDemo(const std::string& color_topic,
                           const std::string& depth_topic,
                           const sensor_msgs::CameraInfo& camera_info,
                           const std::vector<sensor_msgs::Image>& color_images,
                           const std::vector<sensor_msgs::Image>& depth_images,
                           const std::shared_ptr<DemoModel>& demo_model) {
  ResetState();
  color_scrubber_.set_images(color_images);
  depth_scrubber_.set_images(depth_images);
  num_frames_ = depth_images.size();
  camera_info_ = camera_info;

  ss_msgs::ResetSkeletonTrackerRequest reset_req;
  reset_req.rgb_topic = color_topic;
  reset_req.depth_topic = depth_topic;
  reset_req.camera_info = camera_info;
  ss_msgs::ResetSkeletonTrackerResponse reset_res;
  skel_services_.reset.call(reset_req, reset_res);

  demo_model_ = demo_model;
  states_.resize(num_frames_);
}

void DemoRuntime::Step() {
  if (frame_number_ >= num_frames_) {
    ROS_INFO("Reached end of demonstration.");
    return;
  }

  msgs::DemoState prev_state;
  if (frame_number_ > 0) {
    prev_state = states_[frame_number_ - 1];
  }

  msgs::DemoState current_state;

  // Update images
  sensor_msgs::Image color_image;
  sensor_msgs::Image depth_image;
  color_scrubber_.View(frame_number_, &current_color_image_);
  depth_scrubber_.View(frame_number_, &current_depth_image_);

  // Update skeleton state
  // Set skeleton state from annotation if it exists. Otherwise, step through
  // the skeleton tracker.
  if (demo_model_->HasEventAt(msgs::Event::SET_SKELETON_STATE, frame_number_)) {
    msgs::Event skeleton_event;
    demo_model_->EventAt(msgs::Event::SET_SKELETON_STATE, frame_number_,
                         &skeleton_event);
    current_state.nerf_joint_states = skeleton_event.nerf_joint_states;
    skel_services_.nerf_pub.publish(current_state.nerf_joint_states);
  } else {
    // If the previous state was blank (e.g., first frame), then set it to a
    // default skeleton pose.
    if (prev_state.nerf_joint_states.values.size() == 0) {
      skel_services_.nerf_pub.publish(DefaultSkeleton());
      // TODO: no guarantee this initialization occurs before advancing
      ros::spinOnce();
    }
    ss_msgs::AdvanceSkeletonRequest advance_req;
    ss_msgs::AdvanceSkeletonResponse advance_res;
    advance_req.rgb = current_color_image_;
    advance_req.depth = current_depth_image_;
    skel_services_.advance.call(advance_req, advance_res);
    current_state.nerf_joint_states = advance_res.joint_states;
  }

  // Update object states
  // Set the object states from annotation events if they exist. Otherwise, step
  // through all the object trackers.
  // TODO: finish this
  // object_tracker_->update_obsrv(current_depth_image_);
  // object_tracker_->run_once();
  // object_pub_->publish(object_tracker_->current_state_messages());

  PublishViz();

  states_[frame_number_] = current_state;

  ++frame_number_;
}

int DemoRuntime::current_frame_number() const { return frame_number_; }

void DemoRuntime::current_color_image(sensor_msgs::Image* image) const {
  *image = current_color_image_;
}

void DemoRuntime::current_depth_image(sensor_msgs::Image* image) const {
  *image = current_depth_image_;
}

void DemoRuntime::GetState(const int frame_number,
                           task_perception_msgs::DemoState* state) const {
  if (frame_number < 0 || frame_number >= static_cast<int>(states_.size())) {
    ROS_ERROR("Invalid frame number %d (out of %ld)", frame_number,
              states_.size());
    return;
  }
  *state = states_[frame_number];
}

void DemoRuntime::ResetState() {
  object_trackers_.clear();
  frame_number_ = 0;
  states_.clear();
}

void DemoRuntime::PublishViz() {
  viz_.camera_info_pub.publish(camera_info_);
  ros::Time now = ros::Time::now();
  current_color_image_.header.stamp = now;
  current_depth_image_.header.stamp = now;
  viz_.color_pub.publish(current_color_image_);
  viz_.depth_pub.publish(current_depth_image_);
}

}  // namespace pbi
