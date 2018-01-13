#include <string>

#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/AdvanceSkeleton.h"
#include "skin_segmentation_msgs/GetSkeletonState.h"
#include "skin_segmentation_msgs/NerfJointStates.h"
#include "skin_segmentation_msgs/ResetSkeletonTracker.h"
#include "task_perception_msgs/AnnotatorState.h"
#include "task_perception_msgs/Demonstration.h"

#include "task_perception/annotator_server.h"
#include "task_perception/database.h"
#include "task_perception/demo_visualizer.h"
#include "task_perception/skeleton_services.h"

namespace msgs = task_perception_msgs;
namespace ss_msgs = skin_segmentation_msgs;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pbi_annotator_node");
  ros::NodeHandle nh;

  pbi::DemoVisualizer demo_viz;
  demo_viz.camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(
      "pbi_annotator/camera_info", 10, true);
  demo_viz.color_pub =
      nh.advertise<sensor_msgs::Image>("pbi_annotator/image_color", 10, true);
  demo_viz.depth_pub =
      nh.advertise<sensor_msgs::Image>("pbi_annotator/image_depth", 10, true);
  demo_viz.state_pub =
      nh.advertise<msgs::AnnotatorState>("pbi_annotator/state", 10, true);
  demo_viz.objects_pub = nh.advertise<visualization_msgs::Marker>(
      "pbi_annotator/objects", 10, true);

  // Skeleton tracker services
  pbi::SkeletonServices skel_services;
  skel_services.reset =
      nh.serviceClient<ss_msgs::ResetSkeletonTracker>("reset_skeleton_tracker");
  skel_services.advance =
      nh.serviceClient<ss_msgs::AdvanceSkeleton>("advance_skeleton");
  skel_services.get_state =
      nh.serviceClient<ss_msgs::GetSkeletonState>("get_skeleton_state");
  skel_services.nerf_pub =
      nh.advertise<skin_segmentation_msgs::NerfJointStates>("nerf_controls", 1);
  while (ros::ok() &&
         (!skel_services.reset.waitForExistence(ros::Duration(1.0)) ||
          !skel_services.advance.waitForExistence(ros::Duration(1.0)) ||
          !skel_services.get_state.waitForExistence(ros::Duration(1.0)))) {
    ROS_WARN("Waiting for skeleton tracking service");
  }

  // Build database
  const std::string& kDatabaseName("pbi");
  const std::string& kCollection("demonstrations");
  mongodb_store::MessageStoreProxy message_store(nh, kCollection,
                                                 kDatabaseName);
  ros::Publisher demo_pub = nh.advertise<task_perception_msgs::Demonstration>(
      "pbi_annotator/demonstration", 1, true);
  pbi::DemonstrationDb demo_db(&message_store, demo_pub);

  pbi::AnnotatorServer server(demo_viz, skel_services, demo_db);
  server.Start();
  ROS_INFO("Annotator server ready.");

  ros::Subscriber event_sub = nh.subscribe(
      "pbi_annotator/events", 100, &pbi::AnnotatorServer::HandleEvent, &server);

  ros::spin();
  return 0;
}
