#ifndef _PBI_PROGRAM_GENERATOR_H_
#define _PBI_PROGRAM_GENERATOR_H_

#include <map>
#include <string>

#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/Step.h"

#include "task_imitation/collision_checker.h"
#include "task_imitation/program_segment.h"

namespace pbi {
// Generates an executable robot program given a sequence of states extracted
// from a demonstration.
class ProgramGenerator {
 public:
  typedef std::map<std::string, task_perception_msgs::ObjectState>
      ObjectStateIndex;

  ProgramGenerator(moveit::planning_interface::MoveGroup& left_group,
                   moveit::planning_interface::MoveGroup& right_group);
  task_perception_msgs::Program Generate(
      const std::vector<task_perception_msgs::DemoState>& demo_states,
      const ObjectStateIndex& initial_objects);

 private:
  std::vector<ProgramSegment> Segment(
      const std::vector<task_perception_msgs::DemoState>& demo_states);
  void ProcessSegment(const ProgramSegment& state,
                      const ObjectStateIndex& initial_runtime_objects,
                      const ObjectStateIndex& initial_demo_objects);
  void AddGraspStep(const ProgramSegment& segment,
                    const ObjectStateIndex& initial_runtime_objects);
  void AddUngraspStep(const ProgramSegment& segment);
  void AddMoveToStep(const ProgramSegment& segment,
                     const ObjectStateIndex& initial_demo_objects);
  void AddTrajectoryStep(const ProgramSegment& segment,
                         const ObjectStateIndex& initial_runtime_objects);

  int GetMostRecentGraspStep(const std::string& arm_name);
  ros::Duration GetEndTime(const task_perception_msgs::Step& step);

  task_perception_msgs::Program program_;

  // The real-world timestamp of first step we take.
  ros::Time start_time_;

  moveit::planning_interface::MoveGroup& left_group_;
  moveit::planning_interface::MoveGroup& right_group_;
  std::string planning_frame_;

  CollisionChecker collision_checker_;
};

// Gets the initial states of all the objects at the time of the demonstration.
// NOTE: the poses of these objects are in the camera frame. In contrast, the
// poses of "runtime objects" are given in the robot's planning/base frame.
ProgramGenerator::ObjectStateIndex GetInitialDemoObjects(
    const std::vector<task_perception_msgs::DemoState>& demo_states);
}  // namespace pbi

#endif  // _PBI_PROGRAM_GENERATOR_H_
