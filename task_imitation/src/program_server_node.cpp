#include "rapid_robot/camera_interface.h"
#include "rapid_robot/point_cloud_camera.h"
#include "rapid_robot/recorded_point_cloud_camera.h"
#include "ros/ros.h"
#include "task_perception_msgs/GetDemoStates.h"

#include "task_imitation/program_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "program_generator");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::ServiceClient db_client =
      nh.serviceClient<task_perception_msgs::GetDemoStates>("get_demo_states");
  while (!db_client.waitForExistence(ros::Duration(1))) {
    ROS_WARN("Waiting for service get_demo_states.");
  }

  rapid::PointCloudCameraInterface* cam_interface;
  if (argc > 1) {
    std::string bag_path(argv[1]);
    cam_interface = new rapid::RecordedPointCloudCamera();
    static_cast<rapid::RecordedPointCloudCamera*>(cam_interface)
        ->LoadBag(bag_path);
  } else {
    cam_interface = new rapid::PointCloudCamera(
        "/head_mount_kinect/depth_registered/points", "base_link");
  }

  pbi::ProgramServer program_server(db_client, *cam_interface);
  ros::Subscriber event_sub =
      nh.subscribe("pbi_imitation/events", 5, &pbi::ProgramServer::HandleEvent,
                   &program_server);
  program_server.Start();

  ROS_INFO("Task imitation server ready.");
  ros::waitForShutdown();
  delete cam_interface;
  return 0;
}
