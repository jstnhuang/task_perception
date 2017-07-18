#ifndef _PBI_TRACK_H_
#define _PBI_TRACK_H_

#include <memory>
#include <string>

#include "dbot/object_resource_identifier.h"
#include "dbot/tracker/particle_tracker.h"
#include "dbot_ros/object_tracker_publisher.h"
#include "dbot_ros/object_tracker_ros.h"
#include "ros/ros.h"

namespace pbi {
// Represents the tracking of a single object for a period of time.
class Track {
 public:
  Track(const ros::NodeHandle& nh, const dbot::ObjectResourceIdentifier& ori);

  // Step once through the track.
  void Step();

  // Start the tracking from the beginning again.
  void Reset();

  int current_frame_index() const;

 private:
  ros::NodeHandle nh_;
  std::shared_ptr<dbot::ObjectTrackerRos<dbot::ParticleTracker> > tracker_;
  int current_frame_index_;
  dbot::ObjectStatePublisher state_pub_;
};
}  // namespace pbi

#endif  // _PBI_TRACK_H_
