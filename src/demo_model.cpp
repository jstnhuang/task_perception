#include "task_perception/demo_model.h"

#include "ros/ros.h"
#include "task_perception_msgs/Demonstration.h"
#include "task_perception_msgs/Event.h"

using task_perception_msgs::Demonstration;
using task_perception_msgs::Event;

namespace pbi {
DemoModel::DemoModel(const task_perception_msgs::Demonstration& demo)
    : demo_(demo), timeline_(demo_.frame_count) {
  Reindex();
}

void DemoModel::Reindex() {
  timeline_.clear();
  timeline_.resize(demo_.frame_count);
  for (const Event& event : demo_.events) {
    if (event.frame_number < 0 || event.frame_number >= demo_.frame_count) {
      ROS_ERROR("[Reindex] Invalid frame number %d (%d)!", event.frame_number,
                demo_.frame_count);
      continue;
    }
    timeline_[event.frame_number].push_back(event);
  }
}

std::vector<task_perception_msgs::Event> DemoModel::EventsAt(int frame_number) {
  if (frame_number < 0 || frame_number >= demo_.frame_count) {
    ROS_ERROR("[EventsAt] Invalid frame number %d (%d)!", frame_number,
              demo_.frame_count);
    return {};
  }

  return timeline_[frame_number];
}

void DemoModel::AddEvent(const task_perception_msgs::Event& event) {
  if (event.frame_number < 0 || event.frame_number >= demo_.frame_count) {
    ROS_ERROR("[AddEvent] Invalid frame number %d (%d)!", event.frame_number,
              demo_.frame_count);
    return;
  }
  for (Event& evt : timeline_[event.frame_number]) {
    if (evt.type == event.type) {
      evt = event;
      return;
    }
  }
  timeline_[event.frame_number].push_back(event);
}

Demonstration DemoModel::ToMsg() {
  // This copying is necessary because of the de-duping in AddEvent.
  Demonstration demo = demo_;
  for (const auto& frame_events : timeline_) {
    demo.events.insert(demo.events.end(), frame_events.begin(),
                       frame_events.end());
  }
  return demo;
}
}  // namespace pbi
