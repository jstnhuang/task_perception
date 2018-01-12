#include "task_perception/demo_visualizer.h"

#include <vector>

#include "ros/ros.h"

#include "task_perception/object_tracker.h"

namespace pbi {
DemoVisualizer::DemoVisualizer()
    : camera_info_pub(), color_pub(), depth_pub(), state_pub(), objects_pub() {}

void DemoVisualizer::PublishObjects(
    const std::vector<ObjectTracker>& object_trackers) const {
  for (const auto& tracker : object_trackers) {
    // TODO: finish this
  }
}
}  // namespace pbi
