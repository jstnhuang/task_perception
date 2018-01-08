#ifndef _PBI_PARTICLE_TRACKER_BUILDER_H_
#define _PBI_PARTICLE_TRACKER_BUILDER_H_

#include <memory>

#include "dbot/object_resource_identifier.h"
#include "dbot/tracker/particle_tracker.h"
#include "dbot_ros/object_tracker_ros.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

namespace pbi {
// Builds a complete particle tracker, including reading params from the
// parameter server.
class ParticleTrackerBuilder {
 public:
  ParticleTrackerBuilder(const ros::NodeHandle& nh,
                         const sensor_msgs::CameraInfo& camera_info);

  // Sets the object to track.
  void set_object(const dbot::ObjectResourceIdentifier& ori);

  // Builds the particle tracker.
  std::shared_ptr<dbot::ParticleTracker> Build();
  // Builds the ROS version of the particle tracker.
  std::shared_ptr<dbot::ObjectTrackerRos<dbot::ParticleTracker> > BuildRos();

 private:
  ros::NodeHandle nh_;
  sensor_msgs::CameraInfo camera_info_;

  dbot::ObjectResourceIdentifier ori_;
  std::shared_ptr<dbot::CameraData> camera_data_;
};

void BuildOri(const ros::NodeHandle& nh, const std::string& mesh_name,
              dbot::ObjectResourceIdentifier* ori);

std::shared_ptr<dbot::CameraData> BuildCameraData(const ros::NodeHandle& nh);
}  // namespace pbi

#endif  // _PBI_PARTICLE_TRACKER_BUILDER_H_
