#ifndef _PBI_PROGRAM_SERVER_H_
#define _PBI_PROGRAM_SERVER_H_

#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "moveit/move_group_interface/move_group.h"
#include "pr2_actions/gripper.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/Program.h"
#include "tf/transform_listener.h"

namespace pbi {
std::vector<Slice> SliceProgram(const task_perception_msgs::Program& program);
std::vector<geometry_msgs::Pose> SampleTrajectory(
    const std::vector<geometry_msgs::Pose>& traj);

class ProgramServer {
 public:
  ProgramServer(const ros::ServiceClient& db_client);
  void Start();
  void ExecuteImitation(
      const task_perception_msgs::ImitateDemoGoalConstPtr& goal);

 private:
  std::map<std::string, task_perception_msgs::ObjectState> GetObjectPoses(
      const task_perception_msgs::Program& program);
  std::vector<Slice> ComputeSlices(
      const task_perception_msgs::DemoStates& demo_states);

  ros::ServiceClient db_client_;
  moveit::planning_interface::MoveGroup left_group_;
  moveit::planning_interface::MoveGroup right_group_;
  moveit::planning_interface::MoveGroup arms_group_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<task_perception_msgs::ImitateDemoAction>
      action_server_;
  actionlib::SimpleActionClient<dbot_ros_msgs::InitializeObjectAction>
      initialize_object_;
  const std::string planning_frame_;

  ros::Publisher left_traj_pub_;
  ros::Publisher right_traj_pub_;

  pr2_actions::Gripper left_gripper_;
  pr2_actions::Gripper right_gripper_;
  tf::TransformListener tf_listener_;

  ros::Publisher gripper_pub_;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_SERVER_H_
