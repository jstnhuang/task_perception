#ifndef _PBI_PROGRAM_SERVER_H_
#define _PBI_PROGRAM_SERVER_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "boost/optional.hpp"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"

namespace pbi {
class ProgramServer {
 public:
  ProgramServer(const ros::ServiceClient& db_client,
                const std::string& moveit_planning_group);
  void Start();
  void ExecuteImitation(
      const task_perception_msgs::ImitateDemoGoalConstPtr& goal);

 private:
  ros::ServiceClient db_client_;
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
