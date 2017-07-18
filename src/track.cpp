#include "task_perception/track.h"

#include <memory>
#include <string>

#include "dbot/object_resource_identifier.h"
#include "dbot/tracker/particle_tracker.h"
#include "dbot_ros/object_tracker_publisher.h"
#include "ros/ros.h"

#include "task_perception/particle_tracker_builder.h"

namespace pbi {
Track::Track(const ros::NodeHandle& nh,
             const dbot::ObjectResourceIdentifier& ori)
    : nh_(nh), tracker_(), current_frame_index_(0), state_pub_(ori, 0, 255, 0) {
  ParticleTrackerBuilder builder(nh_);
  builder.set_object(ori);
  tracker_ = builder.BuildRos();
}

void Track::Step() {}

void Track::Reset() {}

int Track::current_frame_index() const { return current_frame_index_; }
}  // namespace pbi
