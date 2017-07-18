#ifndef _PBI_PARTICLE_TRACKER_BUILDER_H_
#define _PBI_PARTICLE_TRACKER_BUILDER_H_

#include <memory>

#include "dbot/tracker/particle_tracker.h"
#include "dbot_ros/object_tracker_ros.h"
#include "ros/ros.h"

namespace pbi {
// Builds a complete particle tracker, including reading params from the
// parameter server.
class ParticleTrackerBuilder {
 public:
  ParticleTrackerBuilder(const ros::NodeHandle nh);

  // Sets the objects this particle tracker should track.
  //
  // Args:
  //  object_meshes: The names of the object meshes in .obj format in the object
  //    model directory. E.g., "wrench_1k.obj", "ball.obj".
  void set_object_meshes(const std::vector<std::string>& object_meshes);

  // Builds the particle tracker.
  std::shared_ptr<dbot::ParticleTracker> Build();
  // Builds the ROS version of the particle tracker.
  std::shared_ptr<dbot::ObjectTrackerRos<dbot::ParticleTracker> > BuildRos();

 private:
  ros::NodeHandle nh_;
  std::vector<std::string> object_meshes_;
};
}  // namespace pbi

#endif  // _PBI_PARTICLE_TRACKER_BUILDER_H_
