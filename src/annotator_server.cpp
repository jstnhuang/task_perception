#include "task_perception/annotator_server.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/AnnotatorEvent.h"

namespace msgs = task_perception_msgs;

namespace pbi {
AnnotatorServer::AnnotatorServer(const VideoScrubber& scrubber)
    : scrubber_(scrubber) {}

void AnnotatorServer::Start() {}

void AnnotatorServer::HandleEvent(
    const task_perception_msgs::AnnotatorEvent& event) {
  if (event.type == msgs::AnnotatorEvent::OPEN_BAG) {
    // TODO: load images
  } else if (event.type == msgs::AnnotatorEvent::VIEW_TIME) {
    sensor_msgs::Image image;
    scrubber_.View(event.time, &image);
    // TODO: publish image
  } else {
    ROS_ERROR("Unknown event type: \"%s\"", event.type.c_str());
  }
}
}  // namespace pbi
