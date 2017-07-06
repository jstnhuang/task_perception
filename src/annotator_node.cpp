#include "ros/ros.h"
#include "task_perception/record_video_action_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pbi_annotator_node");
  ros::NodeHandle nh;
  pbi::RecordVideoActionServer record_server;
  record_server.Start();

  ros::spin();
  return 0;
}
