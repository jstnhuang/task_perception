#ifndef _PBI_IK_H_
#define _PBI_IK_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"

namespace pbi {
// Checks for an IK solution. ee_pose is given in planning frame.
bool HasIk(const moveit::planning_interface::MoveGroup& move_group,
           const geometry_msgs::Pose& ee_pose);
}  // namespace pbi

#endif  // _PBI_IK_H_
