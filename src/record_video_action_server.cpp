#include "task_perception/record_video_action_server.h"

#include <string>

#include "actionlib/server/simple_action_server.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/RecordVideoAction.h"
#include "tf/transform_listener.h"

#include "boost/shared_ptr.hpp"

namespace pbi {
RecordVideoActionServer::RecordVideoActionServer()
    : nh_(),
      tf_listener_(),
      color_sub_(nh_.subscribe(kColorTopic, 100,
                               &RecordVideoActionServer::ColorCallback, this)),
      depth_sub_(nh_.subscribe(kDepthTopic, 100,
                               &RecordVideoActionServer::DepthCallback, this)),
      as_(nh_, "record_video",
          boost::bind(&RecordVideoActionServer::Execute, this, _1), false),
      current_bag_() {}

void RecordVideoActionServer::Start() { as_.start(); }

void RecordVideoActionServer::Execute(
    const task_perception_msgs::RecordVideoGoalConstPtr& goal) {
  // Get camera info
  sensor_msgs::CameraInfoConstPtr camera_info =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", nh_,
                                                          ros::Duration(5));
  if (!camera_info) {
    Finish("Timed out reading from camera info topic: " +
           ros::names::resolve(kCameraInfoTopic));
    return;
  }

  current_bag_.reset(new rosbag::Bag);
  try {
    current_bag_->open(goal->bag_path, rosbag::bagmode::Write);
  } catch (rosbag::BagException& ex) {
    Finish("Could not write to bag file: " + goal->bag_path);
    return;
  }
  current_bag_->write(kCameraInfoTopic, ros::Time::now(), *camera_info);

  // Get camera transform
  tf::StampedTransform tf_transform;
  try {
    tf_listener_.lookupTransform(camera_info->header.frame_id, "base_link",
                                 ros::Time(0), tf_transform);
  } catch (tf::TransformException& ex) {
    Finish("Could not get transform from base_link to " +
           camera_info->header.frame_id);
    return;
  }
  geometry_msgs::Transform transform;
  transform.translation.x = tf_transform.getOrigin().x();
  transform.translation.y = tf_transform.getOrigin().y();
  transform.translation.z = tf_transform.getOrigin().z();
  transform.rotation.w = tf_transform.getRotation().w();
  transform.rotation.x = tf_transform.getRotation().x();
  transform.rotation.y = tf_transform.getRotation().y();
  transform.rotation.z = tf_transform.getRotation().z();
  current_bag_->write(kCameraTransformTopic, ros::Time::now(), transform);

  ros::Timer timer = nh_.createTimer(
      ros::Duration(1), &RecordVideoActionServer::PublishFeedback, this);
  while (ros::ok() && !as_.isPreemptRequested()) {
    ros::spinOnce();
  }
  timer.stop();
  Finish("");
}

void RecordVideoActionServer::ColorCallback(const sensor_msgs::Image& image) {
  if (current_bag_) {
    current_bag_->write(kColorTopic, image.header.stamp, image);
  }
}

void RecordVideoActionServer::DepthCallback(const sensor_msgs::Image& image) {
  if (current_bag_) {
    current_bag_->write(kDepthTopic, image.header.stamp, image);
  }
}

void RecordVideoActionServer::PublishFeedback(const ros::TimerEvent& event) {
  if (current_bag_) {
    task_perception_msgs::RecordVideoFeedback feedback;
    feedback.bag_size = current_bag_->getSize();
    as_.publishFeedback(feedback);
  }
}

void RecordVideoActionServer::Finish(const std::string& error) {
  task_perception_msgs::RecordVideoResult result;
  if (error != "") {
    result.error = error;
    ROS_ERROR("%s", result.error.c_str());
    as_.setAborted(result, result.error);
  } else {
    as_.setSucceeded(result);
  }
  current_bag_->close();
  current_bag_.reset();
}
}  // namespace pbi
