#include "task_perception/demo_model.h"

#include <string>
#include <vector>

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
  for (size_t i = 0; i < demo_.events.size(); ++i) {
    const Event& event = demo_.events[i];
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
    std::vector<task_perception_msgs::Event> empty;
    return empty;
  }

  return timeline_[frame_number];
}

std::vector<task_perception_msgs::Event> DemoModel::EventsAt(
    const std::string& event_type, int frame_number) const {
  if (frame_number < 0 || frame_number >= demo_.frame_count) {
    ROS_ERROR("[EventsAt] Invalid frame number %d (%d)!", frame_number,
              demo_.frame_count);
    std::vector<task_perception_msgs::Event> empty;
    return empty;
  }

  std::vector<Event> filtered;
  for (size_t i = 0; i < timeline_[frame_number].size(); ++i) {
    const Event& evt = timeline_[frame_number][i];
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
  for (size_t i = 0; i < timeline_[frame_number].size(); ++i) {
    const Event& evt = timeline_[frame_number][i];
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
  // Check to see if we are replacing an existing event.
  for (size_t i = 0; i < timeline_[event.frame_number].size(); ++i) {
    Event& existing = timeline_[event.frame_number][i];
    // Only replace events of the same type
    if (event.type != existing.type) {
      continue;
    }

    // For object-related events, only replace if the two events pertain to the
    // same object.
    if (event.type == Event::SET_OBJECT_POSE ||
        event.type == Event::SPAWN_OBJECT ||
        event.type == Event::UNSPAWN_OBJECT) {
      if (event.object_name == existing.object_name) {
        existing = event;
        return;
      }
    } else {
      existing = event;
      return;
    }
  }
  // Otherwise, insert a new event
  timeline_[event.frame_number].push_back(event);
}

void DemoModel::DeleteEvent(const Event& event, int frame_number) {
  if (frame_number < 0 || frame_number >= demo_.frame_count) {
    ROS_ERROR("[DeleteEvent] Invalid frame number %d (%d)!", frame_number,
              demo_.frame_count);
    return;
  }

  std::vector<Event> cleaned;
  for (size_t i = 0; i < timeline_[frame_number].size(); ++i) {
    const Event& existing = timeline_[frame_number][i];
    if (event.type != existing.type) {
      cleaned.push_back(existing);
      continue;
    }
    if (event.type == Event::SET_OBJECT_POSE ||
        event.type == Event::SPAWN_OBJECT ||
        event.type == event.UNSPAWN_OBJECT) {
      if (event.object_name != existing.object_name) {
        cleaned.push_back(existing);
        continue;
      }
    } else {
      cleaned.push_back(existing);
      continue;
    }
  }
  timeline_[frame_number] = cleaned;
}

Demonstration DemoModel::ToMsg() const {
  // This copying is necessary because of the de-duping in AddEvent.
  Demonstration demo = demo_;
  demo.events.clear();
  for (size_t i = 0; i < timeline_.size(); ++i) {
    const std::vector<Event>& frame_events = timeline_[i];
    demo.events.insert(demo.events.end(), frame_events.begin(),
                       frame_events.end());
  }
  return demo;
}
}  // namespace pbi
