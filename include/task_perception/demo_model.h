#ifndef _PBI_DEMO_MODEL_H_
#define _PBI_DEMO_MODEL_H_

#include <string>
#include <vector>

#include "task_perception_msgs/Demonstration.h"
#include "task_perception_msgs/Event.h"

namespace pbi {
// A DemoModel wraps a task_perception_msgs::Demonstration.
//
// It indexes the events by time and provides helper methods.
class DemoModel {
 public:
  DemoModel(const task_perception_msgs::Demonstration& demo);
  void Reindex();
  std::vector<task_perception_msgs::Event> EventsAt(int frame_number);
  bool EventAt(const std::string& event_type, int frame_number,
               task_perception_msgs::Event* event);
  bool HasEventAt(const std::string& event_type, int frame_number);
  // Adds an event to the timeline. Replaces any event of the same type at the
  // same time.
  void AddEvent(const task_perception_msgs::Event& event);
  task_perception_msgs::Demonstration ToMsg();

 private:
  task_perception_msgs::Demonstration demo_;
  std::vector<std::vector<task_perception_msgs::Event>> timeline_;
};

}  // namespace pbi

#endif  // _PBI_DEMO_MODEL_H_
