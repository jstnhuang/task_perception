#include "task_perception/demo_visualizer.h"

#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "task_perception_msgs/ObjectState.h"
#include "transform_graph/graph.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_perception/pr2_gripper_model.h"
#include "task_perception/task_perception_context.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using visualization_msgs::Marker;

namespace pbi {
DemoVisualizer::DemoVisualizer()
    : camera_info_pub(),
      color_pub(),
      depth_pub(),
      state_pub(),
      objects_pub(),
      gripper_pub() {}

void DemoVisualizer::ClearObjects(
    const std::vector<msgs::ObjectState>& prev_states) const {
  for (size_t i = 0; i < prev_states.size(); ++i) {
    const msgs::ObjectState& state = prev_states[i];
    Marker marker;
    MakeDeleteMarker(state.name, &marker);
    objects_pub.publish(marker);
  }
}

void DemoVisualizer::PublishObjects(
    const std::vector<msgs::ObjectState>& object_states,
    const std::string& frame_id) const {
  for (size_t i = 0; i < object_states.size(); ++i) {
    const msgs::ObjectState& state = object_states[i];
    Marker marker;
    MakeMeshMarker(state.pose, frame_id, state.name, state.mesh_name, &marker);
    objects_pub.publish(marker);
  }
}

void MakeMeshMarker(const geometry_msgs::Pose& pose,
                    const std::string& frame_id, const std::string& object_name,
                    const std::string& mesh_name, Marker* marker) {
  marker->header.frame_id = frame_id;
  marker->ns = object_name;
  marker->id = 0;
  marker->type = Marker::MESH_RESOURCE;
  marker->action = Marker::ADD;
  marker->pose = pose;
  marker->mesh_resource = "package://object_meshes/object_models/" + mesh_name;
  marker->scale.x = 1;
  marker->scale.y = 1;
  marker->scale.z = 1;
  marker->color.r = 0;
  marker->color.g = 1;
  marker->color.b = 0;
  marker->color.a = 0.5;
}

void MakeDeleteMarker(const std::string& object_name, Marker* marker) {
  marker->ns = object_name;
  marker->id = 0;
  marker->action = Marker::DELETE;
}
}  // namespace pbi
