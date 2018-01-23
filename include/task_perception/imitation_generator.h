#ifndef _PBI_IMITATION_GENERATOR_H_
#define _PBI_IMITATION_GENERATOR_H_

#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/Program.h"

#include "task_perception/task_perception_context.h"

namespace pbi {
class ImitationGenerator {
 public:
  ImitationGenerator();
  void Step(TaskPerceptionContext* context);

 private:
  task_perception_msgs::Program program_;
};
}  // namespace pbi

#endif  // _PBI_IMITATION_GENERATOR_H_
