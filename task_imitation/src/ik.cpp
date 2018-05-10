#include "task_imitation/ik.h"

namespace pbi {
bool HasIk(const moveit::planning_interface::MoveGroup& group,
           const geometry_msgs::Pose& ee_pose) {
  robot_state::RobotState kinematic_state(group.getRobotModel());
  return kinematic_state.setFromIK(
      group.getRobotModel()->getJointModelGroup(group.getName()), ee_pose, 10,
      0.1);
}
}  // namespace pbi
