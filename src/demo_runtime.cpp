#include "task_perception/demo_runtime.h"

#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/AdvanceSkeleton.h"
#include "skin_segmentation_msgs/PredictHands.h"
#include "skin_segmentation_msgs/ResetSkeletonTracker.h"
#include "task_perception_msgs/Demonstration.h"

#include "task_perception/contact_detection.h"
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
                         const SkeletonServices& skel_services,
                         const ros::ServiceClient& predict_hands)
    : viz_(viz),
      skel_services_(skel_services),
      predict_hands_(predict_hands),
      demo_model_(),
      nh_(),
      color_scrubber_(),
      depth_scrubber_(),
      current_color_image_(),
      current_depth_image_(),
      camera_info_(),
      contact_detection_(skel_services, predict_hands),
      last_executed_frame_(-1),
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
  // frame_number is the index of the frame to execute.
  int frame_number = last_executed_frame_ + 1;
  if (frame_number >= num_frames_) {
    ROS_INFO("Reached end of demonstration.");
    return;
  }

  msgs::DemoState prev_state;
  if (frame_number > 0) {
    prev_state = states_[frame_number - 1];
  }

  msgs::DemoState current_state;

  // Update images
  sensor_msgs::Image color_image;
  sensor_msgs::Image depth_image;
  color_scrubber_.View(frame_number, &current_color_image_);
  depth_scrubber_.View(frame_number, &current_depth_image_);

  StepSkeleton(frame_number, prev_state, &current_state.nerf_joint_states);
  StepSpawnUnspawn(frame_number, prev_state);
  StepObjectPose(frame_number, prev_state, &current_state.object_states);
  DetectContact(frame_number, current_state, prev_state);

  states_[frame_number] = current_state;
  ++last_executed_frame_;
  PublishViz();
}

int DemoRuntime::last_executed_frame() const { return last_executed_frame_; }
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

bool DemoRuntime::GetObjectState(const int frame_number,
                                 const std::string& object_name,
                                 msgs::ObjectState* object_state) {
  if (frame_number < 0 || frame_number >= static_cast<int>(states_.size())) {
    ROS_ERROR("Invalid frame number %d (out of %ld)", frame_number,
              states_.size());
    return false;
  }

  for (const auto& os : states_[frame_number].object_states) {
    if (os.object_name == object_name) {
      *object_state = os;
      return true;
    }
  }

  return false;
}

void DemoRuntime::RerunLastFrame() {
  last_executed_frame_ = last_executed_frame_ - 1;
  Step();
}

void DemoRuntime::RemoveSpawnObjectEvent(const std::string& object_name) {
  object_trackers_.erase(object_name);
}
void DemoRuntime::RemoveUnspawnObjectEvent(const std::string& object_name,
                                           const std::string& object_mesh) {
  msgs::DemoState prev_state;
  if (last_executed_frame_ > 0) {
    prev_state = states_[last_executed_frame_ - 1];
  }
  for (const auto& os : prev_state.object_states) {
    if (os.object_name == object_name) {
      if (object_trackers_.find(object_name) == object_trackers_.end()) {
        object_trackers_[object_name].Instantiate(object_name, object_mesh,
                                                  camera_info_);
      }
      object_trackers_[object_name].SetPose(os.object_pose);
      return;
    }
  }
  ROS_ERROR("Unable to find previous state of %s", object_name.c_str());
  object_trackers_.erase(object_name);
}

void DemoRuntime::StepSkeleton(const int frame_number,
                               const msgs::DemoState& prev_state,
                               ss_msgs::NerfJointStates* nerf_joint_states) {
  // Update skeleton state
  // Set skeleton state from annotation if it exists. Otherwise, step through
  // the skeleton tracker.
  if (demo_model_->HasEventAt(msgs::Event::SET_SKELETON_STATE, frame_number)) {
    msgs::Event skeleton_event;
    demo_model_->EventAt(msgs::Event::SET_SKELETON_STATE, frame_number,
                         &skeleton_event);
    *nerf_joint_states = skeleton_event.nerf_joint_states;
    skel_services_.nerf_pub.publish(*nerf_joint_states);
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
    *nerf_joint_states = advance_res.joint_states;
  }
}

void DemoRuntime::StepSpawnUnspawn(
    const int frame_number, const task_perception_msgs::DemoState& prev_state) {
  std::vector<msgs::Event> spawn_events =
      demo_model_->EventsAt(msgs::Event::SPAWN_OBJECT, frame_number);
  for (const msgs::Event& spawn_event : spawn_events) {
    const std::string& name = spawn_event.object_name;
    if (object_trackers_.find(name) != object_trackers_.end()) {
      ROS_INFO("Reusing tracker for object \"%s\"", name.c_str());
      continue;
    }
    object_trackers_[name].Instantiate(name, spawn_event.object_mesh,
                                       camera_info_);
  }

  // Handle unspawn object events
  std::vector<msgs::Event> unspawn_events =
      demo_model_->EventsAt(msgs::Event::UNSPAWN_OBJECT, frame_number);
  for (const msgs::Event& unspawn_event : unspawn_events) {
    const std::string& name = unspawn_event.object_name;
    if (object_trackers_.find(name) == object_trackers_.end()) {
      ROS_ERROR("No SPAWN event found when UNSPAWNing \"%s\"", name.c_str());
      continue;
    }
    object_trackers_.erase(name);
  }
}

void DemoRuntime::StepObjectPose(
    const int frame_number, const msgs::DemoState& prev_state,
    std::vector<msgs::ObjectState>* object_states) {
  // Figure out where all the objects are
  // All SPAWN events should be accompanied by a SET_OBJECT_POSE event for the
  // same object.
  // SET_OBJECT_POSE should not be used if the object is in motion, since we
  // only reset the pose, not the velocity.
  // For each object, the pose is determined using the following algorithm:
  // 1. If a user annotated its location, use that
  // 2. Otherwise, step through the tracker once
  std::vector<msgs::Event> object_pose_events =
      demo_model_->EventsAt(msgs::Event::SET_OBJECT_POSE, frame_number);
  for (auto& kv : object_trackers_) {
    const std::string& object_name = kv.first;
    ObjectTracker& tracker = kv.second;

    msgs::ObjectState object_state;
    object_state.object_name = object_name;
    object_state.mesh_name = tracker.mesh_name();

    bool done = false;
    // Case 1
    for (const msgs::Event& pose_evt : object_pose_events) {
      if (pose_evt.object_name == object_name) {
        object_state.object_pose = pose_evt.object_pose;
        tracker.SetPose(pose_evt.object_pose);
        done = true;
        break;
      }
    }
    if (done) {
      object_states->push_back(object_state);
      continue;
    }

    // Case 2
    for (const auto& prev_obj : prev_state.object_states) {
      if (prev_obj.object_name == object_name) {
        tracker.Step(current_depth_image_);
        tracker.GetPose(&object_state.object_pose);
        done = true;
        break;
      }
    }
    if (!done) {
      ROS_ERROR("%d: Unable to step through object tracking for object %s",
                frame_number, object_name.c_str());
      continue;
    }
    object_states->push_back(object_state);
  }
}

void DemoRuntime::DetectContact(const int frame_number,
                                const msgs::DemoState& current_state,
                                const msgs::DemoState& prev_state) {
  contact_detection_.Predict(current_state, prev_state, current_color_image_,
                             current_depth_image_, camera_info_);
}

void DemoRuntime::ResetState() {
  last_executed_frame_ = -1;
  states_.clear();
}

void DemoRuntime::PublishViz() {
  viz_.camera_info_pub.publish(camera_info_);
  ros::Time now = ros::Time::now();
  current_color_image_.header.stamp = now;
  current_depth_image_.header.stamp = now;
  viz_.color_pub.publish(current_color_image_);
  viz_.depth_pub.publish(current_depth_image_);

  if (last_executed_frame_ > 0) {
    msgs::DemoState prev_state;
    prev_state = states_[last_executed_frame_ - 1];
    viz_.ClearObjects(prev_state.object_states);
  }

  if (last_executed_frame_ >= 0) {
    viz_.PublishObjects(states_[last_executed_frame_].object_states,
                        camera_info_.header.frame_id);
  }
}
}  // namespace pbi
