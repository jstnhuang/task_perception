#include "task_imitation/ik.h"

namespace pbi {
bool HasIk(moveit::planning_interface::MoveGroup& group,
           const geometry_msgs::Pose& ee_pose) {
  robot_state::RobotStatePtr kinematic_state = group.getCurrentState();
  return kinematic_state->setFromIK(
      group.getRobotModel()->getJointModelGroup(group.getName()), ee_pose, 3,
      0.05);
}
}  // namespace pbi
