#ifndef _PBI_MULTI_OBJECT_TRACKER_H_
#define _PBI_MULTI_OBJECT_TRACKER_H_

#include <map>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

namespace pbi {
// The multi-object tracker node was originally designed to hold multiple
// trackers in a map data structure. For some reason, when tracking two objects
// simultaneously, one of the objects would consistently get lost, but both
// objects could be tracked independently. So the hacky solution is to create a
// separate node for each tracker. Amazingly, this worked.
//
// Each node exposes the same services as the multi-object tracker at
// multi_object_track/OBJECT_NAME.
class MultiObjectTracker {
 public:
  MultiObjectTracker();
  void Create(const std::string& name, const std::string& mesh_name,
              const sensor_msgs::CameraInfo& camera_info);
  void Destroy(const std::string& name);
  void SetPose(const std::string& name, const geometry_msgs::Pose& pose);
  void Step(const std::string& name, const sensor_msgs::Image& depth_image);
  void GetPose(const std::string& name, geometry_msgs::Pose* pose);

  std::string GetMeshName(const std::string& object_name);
  bool IsTracking(const std::string& name);
  std::vector<std::string> TrackedObjects();

 private:
  std::map<std::string, std::string> object_meshes_;
  ros::NodeHandle nh_;
  ros::ServiceClient MakeClient(const std::string& object_name);
};
}  // namespace pbi

#endif  // _PBI_MULTI_OBJECT_TRACKER_H_
