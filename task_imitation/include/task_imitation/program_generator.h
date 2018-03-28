#ifndef _PBI_PROGRAM_GENERATOR_H_
#define _PBI_PROGRAM_GENERATOR_H_

#include <map>
#include <string>

#include "boost/optional.hpp"
#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/Step.h"

namespace pbi {

// A ProgramSegment is either:
// - A DemoState where a grasp occurred
// - A DemoState where an ungrasp occurred
// - A sequence of DemoStates where two objects were close together
struct ProgramSegment {
  std::string arm_name;
  std::string type;
  std::vector<task_perception_msgs::DemoState> demo_states;
  // In case of a trajectory, this indicates which object is the target object.
  std::string target_object;
};

// Generates an executable robot program given a sequence of states extracted
// from a demonstration.
class ProgramGenerator {
  typedef std::map<std::string, task_perception_msgs::ObjectState>
      ObjectStateIndex;

 public:
  ProgramGenerator(moveit::planning_interface::MoveGroup& left_group,
                   moveit::planning_interface::MoveGroup& right_group);
  task_perception_msgs::Program Generate(
      const std::vector<task_perception_msgs::DemoState>& demo_states,
      const ObjectStateIndex& initial_objects);

  const static double kGraspDuration;
  const static double kUngraspDuration;

 private:
  std::vector<ProgramSegment> Segment(
      const std::vector<task_perception_msgs::DemoState>& demo_states,
      const ObjectStateIndex& initial_objects);
  boost::optional<ProgramSegment> SegmentGrasp(
      const task_perception_msgs::DemoState& state,
      const task_perception_msgs::DemoState& prev_state,
      const std::string& arm_name);
  boost::optional<ProgramSegment> SegmentUngrasp(
      const task_perception_msgs::DemoState& state,
      const task_perception_msgs::DemoState& prev_state,
      const std::string& arm_name);
  void Step(const task_perception_msgs::DemoState& state,
            const ObjectStateIndex& initial_objects);
  void ProcessStep(const task_perception_msgs::DemoState& state,
                   const ObjectStateIndex& initial_objects,
                   const std::string& arm_name);
  void AddGraspStep(const task_perception_msgs::DemoState& state,
                    const ObjectStateIndex& initial_objects,
                    const std::string& arm_name);
  void AddOrAppendToTrajectoryStep(const task_perception_msgs::DemoState& state,
                                   const ObjectStateIndex& initial_objects,
                                   const std::string& arm_name);
  void AddUngraspStep(const task_perception_msgs::DemoState& state,
                      const ObjectStateIndex& initial_objects,
                      const std::string& arm_name);
  std::string CheckCollisions(
      const task_perception_msgs::ObjectState& object,
      const std::vector<task_perception_msgs::ObjectState>& other_objects);

  // Gets the most recently created step for the given arm.
  // Returns a pointer to the most recent step, or NULL if there was none.
  int GetMostRecentStep(const std::string& arm_name);
  task_perception_msgs::Step GetMostRecentGraspStep(
      const std::string& arm_name);

  geometry_msgs::Pose ComputeRelativePose(
      const task_perception_msgs::DemoState&);

  ros::Duration GetEndTime(const task_perception_msgs::Step& step);

  task_perception_msgs::Program program_;
  task_perception_msgs::DemoState prev_state_;

  // The real-world timestamp of first step we take.
  ros::Time start_time_;

  moveit::planning_interface::MoveGroup& left_group_;
  moveit::planning_interface::MoveGroup& right_group_;
  std::string planning_frame_;

  LazyObjectModel::ObjectModelCache model_cache_;
};

// Add a constant to a vector3.
geometry_msgs::Vector3 InflateScale(const geometry_msgs::Vector3& scale,
                                    double distance);

task_perception_msgs::ObjectState GetObjectState(
    const task_perception_msgs::DemoState& state,
    const std::string& object_name);

// Returns true if two objects are colliding.
bool AreObjectsColliding(const task_perception_msgs::ObjectState& obj1,
                         const task_perception_msgs::ObjectState& obj2);

}  // namespace pbi

#endif  // _PBI_PROGRAM_GENERATOR_H_
