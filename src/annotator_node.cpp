//#include "dbot/camera_data.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/AnnotatorState.h"
#include "tf/transform_broadcaster.h"

#include "task_perception/annotator_server.h"
#include "task_perception/particle_tracker_builder.h"
#include "task_perception/record_video_action_server.h"
#include "task_perception/video_scrubber.h"

namespace msgs = task_perception_msgs;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pbi_annotator_node");
  ros::NodeHandle nh;
  pbi::RecordVideoActionServer record_server;
  record_server.Start();

  ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(
      "pbi_annotator/camera_info", 10, true);
  ros::Publisher color_pub =
      nh.advertise<sensor_msgs::Image>("pbi_annotator/image_color", 10, true);
  ros::Publisher depth_pub =
      nh.advertise<sensor_msgs::Image>("pbi_annotator/image_depth", 10, true);
  ros::Publisher state_pub =
      nh.advertise<msgs::AnnotatorState>("pbi_annotator/state", 10, true);

  // std::shared_ptr<dbot::CameraData> camera_data = pbi::BuildCameraData(nh);

  pbi::AnnotatorServer server(camera_info_pub, color_pub, depth_pub, state_pub);
  server.Start();

  ros::Subscriber event_sub = nh.subscribe(
      "pbi_annotator/events", 100, &pbi::AnnotatorServer::HandleEvent, &server);

  ros::spin();
  return 0;
}
