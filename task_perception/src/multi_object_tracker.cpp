#include "task_perception/multi_object_tracker.h"

#include "dbot_ros_msgs/MultiTrack.h"

using dbot_ros_msgs::MultiTrack;

namespace pbi {
MultiObjectTracker::MultiObjectTracker() : object_meshes_() {}

void MultiObjectTracker::Create(const std::string& name,
                                const std::string& mesh_name,
                                const sensor_msgs::CameraInfo& camera_info) {
  ros::ServiceClient client = MakeClient(name);

  MultiTrack::Request req;
  req.type = MultiTrack::Request::CREATE;
  req.object_name = name;
  req.mesh_name = mesh_name;
  req.camera_info = camera_info;
  MultiTrack::Response res;
  client.call(req, res);

  object_meshes_[name] = mesh_name;
}

void MultiObjectTracker::Destroy(const std::string& name) {
  ros::ServiceClient client = MakeClient(name);

  MultiTrack::Request req;
  req.type = MultiTrack::Request::DESTROY;
  req.object_name = name;
  MultiTrack::Response res;
  client.call(req, res);

  object_meshes_.erase(name);
}

void MultiObjectTracker::SetPose(const std::string& name,
                                 const geometry_msgs::Pose& pose,
                                 const geometry_msgs::Twist& twist) {
  ros::ServiceClient client = MakeClient(name);
  MultiTrack::Request req;
  req.type = MultiTrack::Request::SET_POSE;
  req.object_name = name;
  req.pose = pose;
  req.twist = twist;
  MultiTrack::Response res;
  client.call(req, res);
}

void MultiObjectTracker::Step(const std::string& name,
                              const sensor_msgs::Image& depth_image) {
  ros::ServiceClient client = MakeClient(name);
  MultiTrack::Request req;
  req.type = MultiTrack::Request::STEP;
  req.object_name = name;
  req.depth_image = depth_image;
  MultiTrack::Response res;
  client.call(req, res);
}

void MultiObjectTracker::GetPose(const std::string& name,
                                 geometry_msgs::Pose* pose) {
  ros::ServiceClient client = MakeClient(name);
  MultiTrack::Request req;
  req.type = MultiTrack::Request::GET_POSE;
  req.object_name = name;
  MultiTrack::Response res;
  client.call(req, res);
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

ros::ServiceClient MultiObjectTracker::MakeClient(
    const std::string& object_name) {
  return nh_.serviceClient<dbot_ros_msgs::MultiTrack>("multi_object_track/" +
                                                      object_name);
}
}  // namespace pbi
