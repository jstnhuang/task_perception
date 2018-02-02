#include "task_imitation/program_planner.h"

#include "moveit/move_group_interface/move_group.h"
#include "task_perception_msgs/Program.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "transform_graph/graph.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;

namespace pbi {
ProgramPlanner::ProgramPlanner() : move_group_("right_arm"), tf_listener_() {
  ROS_INFO("Planning reference frame: %s",
           move_group_.getPlanningFrame().c_str());
  ROS_INFO("End-effector frame: %s", move_group_.getEndEffectorLink().c_str());
}

void ProgramPlanner::Plan(const task_perception_msgs::Program& program) {
  tg::Graph graph;
  tf::StampedTransform head_transform;
  tf_listener_.lookupTransform("camera frame", move_group_.getPlanningFrame(),
                               ros::Time(0), head_transform);
  graph.Add("camera", tg::RefFrame(move_group_.getPlanningFrame()),
            head_transform);
  for (size_t i = 0; i < program.steps.size(); ++i) {
    const msgs::Step& step = program.steps[i];
    if (step.action_type != msgs::Step::FOLLOW_TRAJECTORY) {
      continue;
    }
    // TODO: generalize to bimanual manipulation
    if (step.arm != msgs::Step::RIGHT) {
      continue;
    }
  }
}
}  // namespace pbi
