#include "task_perception/multi_object_tracker.h"

#include <map>
#include <string>
#include <vector>

#include "dbot_ros_msgs/MultiTrack.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

using dbot_ros_msgs::MultiTrack;

namespace pbi {
MultiObjectTracker::MultiObjectTracker(const ros::ServiceClient& service)
    : service_(service), object_meshes_() {}

void MultiObjectTracker::Create(const std::string& name,
                                const std::string& mesh_name,
                                const sensor_msgs::CameraInfo& camera_info) {
  MultiTrack::Request req;
  req.type = MultiTrack::Request::CREATE;
  req.object_name = name;
  req.mesh_name = mesh_name;
  req.camera_info = camera_info;
  MultiTrack::Response res;
  service_.call(req, res);

  object_meshes_[name] = mesh_name;
}

void MultiObjectTracker::Destroy(const std::string& name) {
  MultiTrack::Request req;
  req.type = MultiTrack::Request::DESTROY;
  req.object_name = name;
  MultiTrack::Response res;
  service_.call(req, res);

  object_meshes_.erase(name);
}

void MultiObjectTracker::DestroyAll() {
  MultiTrack::Request req;
  req.type = MultiTrack::Request::DESTROY_ALL;
  MultiTrack::Response res;
  service_.call(req, res);

  object_meshes_.clear();
}

void MultiObjectTracker::SetPose(const std::string& name,
                                 const geometry_msgs::Pose& pose) {
  MultiTrack::Request req;
  req.type = MultiTrack::Request::SET_POSE;
  req.object_name = name;
  req.pose = pose;
  MultiTrack::Response res;
  service_.call(req, res);
}

void MultiObjectTracker::Step(const std::string& name,
                              const sensor_msgs::Image& depth_image) {
  MultiTrack::Request req;
  req.type = MultiTrack::Request::STEP;
  req.object_name = name;
  req.depth_image = depth_image;
  MultiTrack::Response res;
  service_.call(req, res);
}

void MultiObjectTracker::GetPose(const std::string& name,
                                 geometry_msgs::Pose* pose) {
  MultiTrack::Request req;
  req.type = MultiTrack::Request::GET_POSE;
  req.object_name = name;
  MultiTrack::Response res;
  service_.call(req, res);
  *pose = res.pose;
}

std::string MultiObjectTracker::GetMeshName(const std::string& object_name) {
  return object_meshes_[object_name];
}

bool MultiObjectTracker::IsTracking(const std::string& name) {
  return object_meshes_.find(name) != object_meshes_.end();
}

std::vector<std::string> MultiObjectTracker::TrackedObjects() {
  std::vector<std::string> result;
  for (std::map<std::string, std::string>::const_iterator it =
           object_meshes_.begin();
       it != object_meshes_.end(); ++it) {
    result.push_back(it->first);
  }
  return result;
}
}  // namespace pbi
