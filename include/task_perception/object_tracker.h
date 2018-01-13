#ifndef _PBI_OBJECT_TRACKER_H_
#define _PBI_OBJECT_TRACKER_H_

#include <memory>
#include <string>

#include "dbot/tracker/particle_tracker.h"
#include "dbot_ros/object_tracker_ros.h"
#include "dbot_ros_msgs/ObjectState.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

namespace pbi {
class ObjectTracker {
 public:
  ObjectTracker();
  void Instantiate(const std::string& name, const std::string& mesh_path,
                   const sensor_msgs::CameraInfo& camera_info);
  void SetPose(const geometry_msgs::Pose& pose);
  void Step(const sensor_msgs::Image& depth);
  void GetPose(geometry_msgs::PoseStamped* pose_stamped) const;
  void GetPose(geometry_msgs::Pose* pose) const;
  std::string name() const;
  std::string mesh_name() const;

 private:
  std::string name_;
  std::string mesh_name_;

  // Strangely, annotator_node fails to link if we have a
  // dbot::ObjectResourceIdentifier as a member.
  std::string mesh_package_;
  std::string mesh_dir_;

  ros::NodeHandle nh_;

  std::shared_ptr<dbot::ObjectTrackerRos<dbot::ParticleTracker> >
      object_tracker_;
};
}  // namespace pbi

#endif  // _PBI_OBJECT_TRACKER_H_
