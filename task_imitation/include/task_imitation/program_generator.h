#ifndef _PBI_PROGRAM_GENERATOR_H_
#define _PBI_PROGRAM_GENERATOR_H_

#include <map>
#include <string>

#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception/object_model_cache.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/Step.h"
#include "task_utils/pr2_gripper_viz.h"

#include "task_imitation/collision_checker.h"
#include "task_imitation/obb.h"
#include "task_imitation/program_segment.h"

namespace pbi {
// Generates an executable robot program given a sequence of states extracted
// from a demonstration.
class ProgramGenerator {
 public:
  typedef std::map<std::string, task_perception_msgs::ObjectState>
      ObjectStateIndex;

  ProgramGenerator(moveit::planning_interface::MoveGroup& left_group,
                   moveit::planning_interface::MoveGroup& right_group,
                   ObjectModelCache* model_cache,
                   const Pr2GripperViz& gripper_viz);
  task_perception_msgs::Program Generate(
      const std::vector<task_perception_msgs::DemoState>& demo_states,
      const ObjectStateIndex& initial_objects, const Obb& table);

 private:
  std::vector<ProgramSegment> Segment(
      const std::vector<task_perception_msgs::DemoState>& demo_states);
  void ProcessSegment(const ProgramSegment& state,
                      const ObjectStateIndex& initial_runtime_objects,
                      const ObjectStateIndex& initial_demo_objects,
                      const Obb& table);
  void AddGraspStep(const ProgramSegment& segment,
                    const ObjectStateIndex& initial_runtime_objects,
                    const Obb& table);
  void AddUngraspStep(const ProgramSegment& segment);
  void AddMoveToStep(const ProgramSegment& segment,
                     const ObjectStateIndex& initial_demo_objects,
                     const ObjectStateIndex& initial_runtime_objects);
  void AddTrajectoryStep(const ProgramSegment& segment,
                         const ObjectStateIndex& initial_demo_objects,
                         const ObjectStateIndex& initial_runtime_objects);

  int GetMostRecentGraspStep(const std::string& arm_name);

  // Checks for an IK solution. ee_pose is given in planning frame.
  bool HasIk(const std::string& arm_name, const geometry_msgs::Pose& ee_pose);

  task_perception_msgs::Program program_;

  // The real-world timestamp of first step we take.
  ros::Time start_time_;

  moveit::planning_interface::MoveGroup& left_group_;
  moveit::planning_interface::MoveGroup& right_group_;
  std::string planning_frame_;

  ObjectModelCache* model_cache_;
  CollisionChecker collision_checker_;
  const Pr2GripperViz& gripper_viz_;
};

// Gets the initial states of all the objects at the time of the demonstration.
// NOTE: the poses of these objects are in the camera frame. In contrast, the
// poses of "runtime objects" are given in the robot's planning/base frame.
ProgramGenerator::ObjectStateIndex GetInitialDemoObjects(
    const std::vector<task_perception_msgs::DemoState>& demo_states);

// Gets the pose of a target object. If the object is circular, it will be
// rotated so that its x-axis points in the x direction of the planning
// frame.
geometry_msgs::Pose GetTargetPose(const LazyObjectModel& model);
}  // namespace pbi

#endif  // _PBI_PROGRAM_GENERATOR_H_
