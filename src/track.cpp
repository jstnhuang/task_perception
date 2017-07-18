#include "task_perception/track.h"

#include <memory>
#include <string>

#include "dbot/object_resource_identifier.h"
#include "dbot/tracker/particle_tracker.h"
#include "dbot_ros/object_tracker_publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "task_perception/particle_tracker_builder.h"
#include "task_perception/video_scrubber.h"

namespace pbi {
Track::Track(const ros::NodeHandle& nh,
             const dbot::ObjectResourceIdentifier& ori,
             VideoScrubber* depth_scrubber)
    : nh_(nh),
      tracker_(),
      depth_scrubber_(depth_scrubber),
      current_frame_index_(0),
      state_pub_(ori, 0, 255, 0) {
  ParticleTrackerBuilder builder(nh_);
  builder.set_object(ori);
  tracker_ = builder.BuildRos();
}

void Track::Step() {
  if (current_frame_index_ >= depth_scrubber_->num_images()) {
    return;
  }
  sensor_msgs::Image image;
  depth_scrubber_->View(current_frame_index_, &image);
  tracker_->update_obsrv(image);
  current_frame_index_ += 1;
}

void Track::Reset() {
  current_frame_index_ = 0;

  // Wait for initialization
}

int Track::current_frame_index() const { return current_frame_index_; }
}  // namespace pbi
