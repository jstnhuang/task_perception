#include "task_perception/demo_model.h"

#include <string>

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
    AddEvent(event);
  }
}

std::vector<task_perception_msgs::Event> DemoModel::EventsAt(
    int frame_number) const {
  if (frame_number < 0 || frame_number >= demo_.frame_count) {
    ROS_ERROR("[EventsAt] Invalid frame number %d (%d)!", frame_number,
              demo_.frame_count);
    return {};
  }

  return timeline_[frame_number];
}

std::vector<task_perception_msgs::Event> DemoModel::EventsAt(
    const std::string& event_type, int frame_number) const {
  if (frame_number < 0 || frame_number >= demo_.frame_count) {
    ROS_ERROR("[EventsAt] Invalid frame number %d (%d)!", frame_number,
              demo_.frame_count);
    return {};
  }

  std::vector<Event> filtered;
  for (const Event& evt : timeline_[frame_number]) {
    if (evt.type == event_type) {
      filtered.push_back(evt);
    }
  }

  return filtered;
}

bool DemoModel::EventAt(const std::string& event_type, int frame_number,
                        Event* event) const {
  if (frame_number < 0 || frame_number >= demo_.frame_count) {
    ROS_ERROR("[HasEventTypeAt] Invalid frame number %d (%d)!", frame_number,
              demo_.frame_count);
    return false;
  }
  for (const Event& evt : timeline_[frame_number]) {
    if (evt.type == event_type) {
      *event = evt;
      return true;
    }
  }
  return false;
}

bool DemoModel::HasEventAt(const std::string& event_type,
                           int frame_number) const {
  Event unused_event;
  return EventAt(event_type, frame_number, &unused_event);
}

void DemoModel::AddEvent(const task_perception_msgs::Event& event) {
  if (event.frame_number < 0 || event.frame_number >= demo_.frame_count) {
    ROS_ERROR("[AddEvent] Invalid frame number %d (%d)!", event.frame_number,
              demo_.frame_count);
    return;
  }
  for (Event& evt : timeline_[event.frame_number]) {
    if (event.type != Event::SET_OBJECT_POSE && evt.type == event.type) {
      evt = event;
      return;
    }
    if (event.type == Event::SET_OBJECT_POSE && evt.type == event.type &&
        evt.object_name == event.object_name) {
      evt = event;
      return;
    }
  }
  timeline_[event.frame_number].push_back(event);
}

void DemoModel::DeleteEvent(const Event& event, int frame_number) {
  if (frame_number < 0 || frame_number >= demo_.frame_count) {
    ROS_ERROR("[DeleteEvent] Invalid frame number %d (%d)!", frame_number,
              demo_.frame_count);
    return;
  }

  std::vector<Event> cleaned;
  for (const Event& evt : timeline_[frame_number]) {
    if (event.type != Event::SET_OBJECT_POSE && evt.type != event.type) {
      cleaned.push_back(evt);
    }
    if (event.type == Event::SET_OBJECT_POSE &&
        (evt.type != event.type || evt.object_name != event.object_name)) {
      cleaned.push_back(evt);
    }
  }
  timeline_[frame_number] = cleaned;
}

Demonstration DemoModel::ToMsg() const {
  // This copying is necessary because of the de-duping in AddEvent.
  Demonstration demo = demo_;
  for (const auto& frame_events : timeline_) {
    demo.events.insert(demo.events.end(), frame_events.begin(),
                       frame_events.end());
  }
  return demo;
}
}  // namespace pbi
