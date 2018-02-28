#ifndef _PBI_DEMO_VISUALIZER_H_
#define _PBI_DEMO_VISUALIZER_H_

#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "task_perception_msgs/ObjectState.h"
#include "visualization_msgs/Marker.h"

#include "task_perception/task_perception_context.h"

namespace pbi {
class DemoVisualizer {
 public:
  DemoVisualizer();
  void ClearObjects(
      const std::vector<task_perception_msgs::ObjectState>& prev_states) const;
  void PublishObjects(
      const std::vector<task_perception_msgs::ObjectState>& object_states,
      const std::string& frame_id) const;

  ros::Publisher camera_info_pub;
  ros::Publisher color_pub;
  ros::Publisher depth_pub;
  ros::Publisher state_pub;
  ros::Publisher objects_pub;
  ros::Publisher gripper_pub;
};

void MakeMeshMarker(const geometry_msgs::Pose& pose,
                    const std::string& frame_id, const std::string& object_name,
                    const std::string& mesh_name,
                    visualization_msgs::Marker* marker);

void MakeDeleteMarker(const std::string& object_name,
                      visualization_msgs::Marker* marker);
}  // namespace pbi

#endif  // _PBI_DEMO_VISUALIZER_H_
