#ifndef _PBI_PROGRAM_GENERATOR_H_
#define _PBI_PROGRAM_GENERATOR_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "task_perception/task_perception_context.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"

#include "task_imitation/grasp_planner.h"

namespace pbi {
// Generates a Program given a sequence of DemoStates.
class ProgramGenerator {
 public:
  ProgramGenerator();
  void Step(const task_perception_msgs::DemoState& state);
  task_perception_msgs::Program program() const;

  const static double kGraspDuration;
  const static double kUngraspDuration;

 private:
  void ProcessContact(const task_perception_msgs::DemoState& state,
                      const std::string& arm_name);

  // Gets the most recently created step for the given arm.
  // Returns a pointer to the most recent step, or NULL if there was none.
  int GetMostRecentStep(const std::string& arm_name);

  void GetObjectState(const task_perception_msgs::DemoState& state,
                      const std::string& object_name,
                      task_perception_msgs::ObjectState* object_state);

  geometry_msgs::Pose ComputeRelativePose(
      const task_perception_msgs::DemoState&);

  ros::Duration GetEndTime(const task_perception_msgs::Step& step);

  task_perception_msgs::Program program_;
  task_perception_msgs::DemoState prev_state_;

  // The real-world timestamp of first step we take.
  ros::Time start_time_;
  GraspPlanner grasp_planner_;
  TaskPerceptionContext::ObjectModelCache* object_models_;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_GENERATOR_H_
