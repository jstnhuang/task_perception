#ifndef _PBI_HAND_STATE_MACHINE_H_
#define _PBI_HAND_STATE_MACHINE_H_

#include <string>
#include <vector>

#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/HandState.h"

#include "task_imitation/collision_checker.h"
#include "task_imitation/program_segment.h"

namespace pbi {
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

  std::string InferCollidee(
      const task_perception_msgs::DemoState& demo_state,
      const std::vector<std::string>& collidees,
      const task_perception_msgs::ObjectState& held_object,
      const std::string& current_target);

  const std::vector<task_perception_msgs::DemoState>& demo_states_;
  std::string arm_name_;
  const CollisionChecker& collision_checker_;
  std::vector<ProgramSegment>* segments_;

  State state_;
  int index_;
  ProgramSegment working_move_;
  ProgramSegment working_traj_;
};

// Precondition: the demo state is of a double collision (the two hands are
// holding objects and the objects are colliding).
// Computes which one has the longest path.
std::string InferDoubleCollisionTarget(
    const std::vector<task_perception_msgs::DemoState>& demo_states,
    int start_index, const CollisionChecker& collision_checker);

}  // namespace pbi

#endif  // _PBI_HAND_STATE_MACHINE_H_
