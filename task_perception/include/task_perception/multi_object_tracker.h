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
class MultiObjectTracker {
 public:
  explicit MultiObjectTracker(const ros::ServiceClient& service);
  void Create(const std::string& name, const std::string& mesh_name,
              const sensor_msgs::CameraInfo& camera_info);
  void Destroy(const std::string& name);
  void DestroyAll();
  void SetPose(const std::string& name, const geometry_msgs::Pose& pose);
  void Step(const std::string& name, const sensor_msgs::Image& depth_image);
  void GetPose(const std::string& name, geometry_msgs::Pose* pose);

  std::string GetMeshName(const std::string& object_name);
  bool IsTracking(const std::string& name);
  std::vector<std::string> TrackedObjects();

 private:
  ros::ServiceClient service_;
  std::map<std::string, std::string> object_meshes_;
};
}  // namespace pbi

#endif  // _PBI_MULTI_OBJECT_TRACKER_H_
