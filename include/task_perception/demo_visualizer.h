#ifndef _PBI_DEMO_VISUALIZER_H_
#define _PBI_DEMO_VISUALIZER_H_

#include <vector>

#include "ros/ros.h"

#include "task_perception/object_tracker.h"

namespace pbi {
class DemoVisualizer {
 public:
  DemoVisualizer();
  void PublishObjects(const std::vector<ObjectTracker>& object_trackers) const;

  ros::Publisher camera_info_pub;
  ros::Publisher color_pub;
  ros::Publisher depth_pub;
  ros::Publisher state_pub;
  ros::Publisher objects_pub;
};
}  // namespace pbi

#endif  // _PBI_DEMO_VISUALIZER_H_
