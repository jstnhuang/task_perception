// Processes a bag file recorded using record_demonstration.launch.
// You must run this on the robot without changing the camera pose.
//
// This performs the following:
// - Trim the first and last few seconds
// - Synchronize the color and depth images and assign them the same timestamp
// - Remove all but the first CameraInfo message.
// - Add the transform from base_link to the camera frame.
//
// The result is saved to a new bag file.

#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "message_filters/cache.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_utils/bag_utils.h"
#include "tf/transform_listener.h"

using sensor_msgs::Image;
typedef message_filters::sync_policies::ApproximateTime<Image, Image> MyPolicy;

class BagWriter {
 public:
  BagWriter(const std::string& output_path);
  void ImageCallback(const sensor_msgs::ImageConstPtr& color,
                     const sensor_msgs::ImageConstPtr& depth);
  void WriteCameraInfo(const sensor_msgs::CameraInfo& camera_info,
                       const ros::Time& time);
  void WriteCameraPose(const geometry_msgs::Pose& camera_pose,
                       const ros::Time& time);
  void Close();

 private:
  rosbag::Bag output_bag_;
};

BagWriter::BagWriter(const std::string& output_path)
    : output_bag_(output_path, rosbag::bagmode::Write) {}

void BagWriter::ImageCallback(const sensor_msgs::ImageConstPtr& color,
                              const sensor_msgs::ImageConstPtr& depth) {
  ros::Time timestamp = depth->header.stamp;
  output_bag_.write("rgb_image", timestamp, color);
  output_bag_.write("depth_image", timestamp, depth);
}

void BagWriter::WriteCameraInfo(const sensor_msgs::CameraInfo& camera_info,
                                const ros::Time& time) {
  output_bag_.write("camera_info", time, camera_info);
}

void BagWriter::WriteCameraPose(const geometry_msgs::Pose& camera_pose,
                                const ros::Time& time) {
  output_bag_.write("camera_pose", time, camera_pose);
}

void BagWriter::Close() { output_bag_.close(); }

int main(int argc, char** argv) {
  ros::init(argc, argv, "process_bag");
  ros::NodeHandle nh;
  tf::TransformListener tf_listener;

  if (argc < 3) {
    std::cout
        << "Usage: rosrun task_perception process_bag INPUT.bag OUTPUT.bag"
        << std::endl;
    return 1;
  }
  std::string input_path(argv[1]);
  rosbag::Bag input_bag(input_path, rosbag::bagmode::Read);

  // Get info
  double front_trim;
  double back_trim;
  if (!ros::param::get("bag_trim_front_seconds", front_trim)) {
    ROS_ERROR("Missing ROS param bag_trim_front_seconds");
    return 1;
  }
  if (!ros::param::get("bag_trim_back_seconds", back_trim)) {
    ROS_ERROR("Missing ROS param bag_trim_back_seconds");
    return 1;
  }

  sensor_msgs::CameraInfo camera_info;
  if (!pbi::GetCameraInfo(input_bag, &camera_info)) {
    ROS_ERROR("Bag file is missing CameraInfo message.");
    return 1;
  }

  std::string color_topic(""), depth_topic("");
  if (!pbi::GetImageTopics(input_bag, &color_topic, &depth_topic)) {
    ROS_ERROR("Unabe to determine RGB and/or depth topics.");
    return 1;
  }

  ROS_INFO("Color topic: %s, depth topic: %s", color_topic.c_str(),
           depth_topic.c_str());
  std::vector<std::string> topics;
  topics.push_back(color_topic);
  topics.push_back(depth_topic);

  rosbag::View view(input_bag);
  ros::Time start_time = view.getBeginTime() + ros::Duration(front_trim);
  ros::Time end_time = view.getEndTime() - ros::Duration(back_trim);
  rosbag::View trimmed_view(input_bag, rosbag::TopicQuery(topics), start_time,
                            end_time);

  // Get transform from base link to camera frame.
  while (ros::ok() &&
         !tf_listener.waitForTransform(camera_info.header.frame_id, "base_link",
                                       ros::Time(0), ros::Duration(5.0))) {
    ROS_WARN("Waiting for transform from base_link to %s",
             camera_info.header.frame_id.c_str());
  }
  tf::StampedTransform camera_frame;
  tf_listener.lookupTransform(camera_info.header.frame_id, "base_link",
                              ros::Time(0), camera_frame);
  geometry_msgs::Pose camera_pose;
  camera_pose.position.x = camera_frame.getOrigin().x();
  camera_pose.position.y = camera_frame.getOrigin().y();
  camera_pose.position.z = camera_frame.getOrigin().z();
  camera_pose.orientation.w = camera_frame.getRotation().w();
  camera_pose.orientation.x = camera_frame.getRotation().x();
  camera_pose.orientation.y = camera_frame.getRotation().y();
  camera_pose.orientation.z = camera_frame.getRotation().z();

  // Start writing output
  std::string output_path(argv[2]);
  BagWriter output_writer(output_path);

  output_writer.WriteCameraInfo(camera_info, start_time);
  output_writer.WriteCameraPose(camera_pose, start_time);

  message_filters::Cache<Image> rgb_cache(100);
  message_filters::Cache<Image> depth_cache(100);
  message_filters::Synchronizer<MyPolicy> sync(MyPolicy(100), rgb_cache,
                                               depth_cache);
  sync.registerCallback(&BagWriter::ImageCallback, &output_writer);

  for (rosbag::View::const_iterator it = trimmed_view.begin(); it != view.end();
       ++it) {
    if (it->getTopic() == color_topic) {
      rgb_cache.add(it->instantiate<sensor_msgs::Image>());
    } else if (it->getTopic() == depth_topic) {
      depth_cache.add(it->instantiate<sensor_msgs::Image>());
    }
  }

  output_writer.Close();

  return 0;
}
