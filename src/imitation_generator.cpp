// Given a demonstration as a sequence of DemoStates, generates a program to
// imitate the demonstration.
#include "task_perception/imitation_generator.h"

#include <string>

#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/Step.h"

namespace msgs = task_perception_msgs;

namespace pbi {
ImitationGenerator::ImitationGenerator() : program_() {}

void ImitationGenerator::Step(TaskPerceptionContext* context) {}
}  // namespace pbi
