#include "task_perception/object_tracker.h"

#include <memory>
#include <string>

#include "dbot/object_resource_identifier.h"
#include "dbot_ros/object_tracker_ros.h"
#include "dbot_ros/util/interactive_marker_initializer.h"
#include "dbot_ros/util/ros_interface.h"
#include "dbot_ros_msgs/ObjectState.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include "task_perception/particle_tracker_builder.h"

namespace pbi {
ObjectTracker::ObjectTracker()
    : name_(""), mesh_name_(""), nh_(), object_tracker_(), object_init_() {}

void ObjectTracker::Instantiate(const std::string& name,
                                const std::string& mesh_name,
                                const sensor_msgs::CameraInfo& camera_info) {
  name_ = name;
  mesh_name_ = mesh_name;

  dbot::ObjectResourceIdentifier ori;

  pbi::ParticleTrackerBuilder object_tracker_builder(nh_, camera_info);
  pbi::BuildOri(nh_, mesh_name, &ori);
  mesh_package_ = ori.package();
  mesh_dir_ = ori.directory();
  object_tracker_builder.set_object(ori);
  object_tracker_.reset();
  object_tracker_ = object_tracker_builder.BuildRos();
  object_init_.reset(
      new opi::InteractiveMarkerInitializer(camera_info.header.frame_id));
}

void ObjectTracker::SetInitialPose() {
  geometry_msgs::Pose obj_init_pose;
  obj_init_pose.position.z = 1;
  obj_init_pose.orientation.w = 1;
  object_init_->set_object(mesh_package_, mesh_dir_, mesh_name_, obj_init_pose,
                           false);
  object_init_->wait_for_object_poses();
  geometry_msgs::Pose initial_pose = object_init_->poses()[0];
  object_tracker_->tracker()->initialize(
      {ri::to_pose_velocity_vector(initial_pose)});
  ROS_INFO_STREAM("Set initial pose to " << initial_pose);
}

void ObjectTracker::Step(const sensor_msgs::Image& depth) {
  if (!object_tracker_) {
    ROS_ERROR("Object tracker for \"%s\" not instantiated", name_.c_str());
    return;
  }
  object_tracker_->update_obsrv(depth);
}

void ObjectTracker::GetPose(geometry_msgs::PoseStamped* pose_stamped) const {
  if (!object_tracker_) {
    ROS_ERROR("Object tracker for \"%s\" not instantiated", name_.c_str());
    return;
  }
  *pose_stamped = object_tracker_->current_pose();
}

std::string ObjectTracker::name() const { return name_; }
std::string ObjectTracker::mesh_name() const { return mesh_name_; }
}  // namespace pbi
