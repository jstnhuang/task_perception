#ifndef _PBI_PROGRAM_GENERATOR_H_
#define _PBI_PROGRAM_GENERATOR_H_

#include <map>
#include <string>

#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/Step.h"

#include "task_imitation/collision_checker.h"
#include "task_imitation/program_constants.h"

namespace pbi {

// A ProgramSegment is either:
// - A DemoState where a grasp occurred
// - A DemoState where an ungrasp occurred
// - A sequence of DemoStates where two objects were close together
struct ProgramSegment {
  std::string arm_name;
  std::string type;
  // If MoveTo, then demo_state[0] is the state that the MoveTo started and
  // demo_state[1] is state when the MoveTo ended.
  std::vector<task_perception_msgs::DemoState> demo_states;
  // In case of a trajectory, this indicates which object is the target object.
  std::string target_object;
};

// State machine for segmenting a demonstration.
// This is designed to be run interleaved with a state machine for the other
// hand.
class HandStateMachine {
 public:
  HandStateMachine(
      const std::vector<task_perception_msgs::DemoState>& demo_states,
      const std::string& arm_name, const CollisionChecker& collision_checker,
      std::vector<ProgramSegment>* segments);
  bool Step();

 private:
  enum State { NONE, FREE_GRASP, STATIONARY_COLLISION, DOUBLE_COLLISION };
  void NoneState(const task_perception_msgs::DemoState& demo_state);
  void FreeGraspState(const task_perception_msgs::DemoState& demo_state);
  void StationaryCollisionState(
      const task_perception_msgs::DemoState& demo_state);
  void DoubleCollisionState(const task_perception_msgs::DemoState& demo_state);

  task_perception_msgs::HandState GetHand(
      const task_perception_msgs::DemoState& demo_state);
  task_perception_msgs::HandState GetOtherHand(
      const task_perception_msgs::DemoState& demo_state);
  task_perception_msgs::HandState GetPrevHand();

  ProgramSegment NewGraspSegment();
  ProgramSegment NewUngraspSegment();
  ProgramSegment NewMoveToSegment();
  ProgramSegment NewTrajSegment();

  const std::vector<task_perception_msgs::DemoState>& demo_states_;
  std::string arm_name_;
  const CollisionChecker& collision_checker_;
  std::vector<ProgramSegment>* segments_;

  State state_;
  int index_;
  ProgramSegment working_move_;
  ProgramSegment working_traj_;
};

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

task_perception_msgs::ObjectState GetObjectState(
    const task_perception_msgs::DemoState& state,
    const std::string& object_name);

// Precondition: the demo state is of a double collision (the two hands are
// holding objects and the objects are colliding).
// Computes which one has the longest path.
std::string InferDoubleCollisionTarget(
    const std::vector<task_perception_msgs::DemoState>& demo_states,
    int start_index, const CollisionChecker& collision_checker);

// Gets the initial states of all the objects at the time of the demonstration.
// NOTE: the poses of these objects are in the camera frame. In contrast, the
// poses of "runtime objects" are given in the robot's planning/base frame.
ProgramGenerator::ObjectStateIndex GetInitialDemoObjects(
    const std::vector<task_perception_msgs::DemoState>& demo_states);
}  // namespace pbi

#endif  // _PBI_PROGRAM_GENERATOR_H_
