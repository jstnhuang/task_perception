#ifndef _PBI_PROGRAM_SERVER_H_
#define _PBI_PROGRAM_SERVER_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"
#include "task_db/demo_states_db.h"
#include "task_perception_msgs/ImitateDemoAction.h"

namespace pbi {
class ProgramServer {
 public:
  ProgramServer(const DemoStatesDb& demo_states_db,
                const std::string& moveit_planning_group);
  void Start();
  void ExecuteImitation(
      const task_perception_msgs::ImitateDemoGoalConstPtr& goal);

 private:
  DemoStatesDb demo_states_db_;
  moveit::planning_interface::MoveGroup move_group_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<task_perception_msgs::ImitateDemoAction>
      action_server_;
  actionlib::SimpleActionClient<dbot_ros_msgs::InitializeObjectAction>
      initialize_object_;
  const std::string planning_frame_;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_SERVER_H_
