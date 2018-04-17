#ifndef _PBI_PROGRAM_SEGMENT_H_
#define _PBI_PROGRAM_SEGMENT_H_

#include <string>
#include <vector>

#include "task_perception_msgs/DemoState.h"

namespace pbi {
// A ProgramSegment is either:
// - A DemoState where a grasp occurred
// - A DemoState where an ungrasp occurred
// - Two DemoStates to move an object relative to a target object
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

ProgramSegment NewGraspSegment();
ProgramSegment NewUngraspSegment();
ProgramSegment NewMoveToSegment();
ProgramSegment NewTrajSegment();
}  // namespace pbi

#endif  // _PBI_PROGRAM_SEGMENT_H_
