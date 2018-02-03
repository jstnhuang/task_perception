#include "ros/ros.h"
#include "task_perception_msgs/GetDemoStates.h"

#include "task_imitation/program_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "program_generator");
  ros::NodeHandle nh;

  ros::ServiceClient db_client =
      nh.serviceClient<task_perception_msgs::GetDemoStates>("get_demo_states");
  while (!db_client.waitForExistence(ros::Duration(1))) {
    ROS_WARN("Waiting for service get_demo_states.");
  }

  pbi::ProgramServer program_server(db_client, "right_arm");
  program_server.Start();

  ROS_INFO("Task imitation server ready.");
  ros::spin();
  return 0;
}
