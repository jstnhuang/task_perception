#include "task_perception/demo_visualizer.h"

#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "rapid_viz/axes_markers.h"
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
using visualization_msgs::MarkerArray;

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

    std::vector<Marker> del_axes = MakeDeleteAxesMarkers(AxesName(i));
    for (size_t j = 0; j < del_axes.size(); ++j) {
      objects_pub.publish(del_axes[j]);
    }
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

    MarkerArray axes =
        rapid::AxesMarkerArray(AxesName(i), frame_id, state.pose, 0.05);
    for (size_t j = 0; j < axes.markers.size(); ++j) {
      objects_pub.publish(axes.markers[j]);
    }
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

std::string AxesName(size_t obj_index) {
  std::stringstream ss;
  ss << "axes_" << obj_index;
  return ss.str();
}

std::vector<Marker> MakeDeleteAxesMarkers(const std::string& ns) {
  std::vector<Marker> result(3);
  for (int i = 0; i < 3; ++i) {
    result[i].ns = ns;
    result[i].id = i;
    result[i].action = Marker::DELETE;
  }
  return result;
}
}  // namespace pbi
