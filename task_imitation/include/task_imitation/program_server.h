#ifndef _PBI_PROGRAM_SERVER_H_
#define _PBI_PROGRAM_SERVER_H_

#include <map>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/Program.h"
#include "tf/transform_listener.h"

#include "task_imitation/program_executor.h"

namespace pbi {
class ProgramServer {
 public:
  ProgramServer(const ros::ServiceClient& db_client);
  void Start();
  void ExecuteImitation(
      const task_perception_msgs::ImitateDemoGoalConstPtr& goal);

 private:
  std::map<std::string, task_perception_msgs::ObjectState> GetObjectPoses(
      const task_perception_msgs::DemoStates& demo_states);

  ros::ServiceClient db_client_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<task_perception_msgs::ImitateDemoAction>
      action_server_;
  actionlib::SimpleActionClient<dbot_ros_msgs::InitializeObjectAction>
      initialize_object_;

  ProgramExecutor executor_;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_SERVER_H_
