#include "task_perception/pcl_utils.h"

#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "task_perception/pcl_typedefs.h"

namespace pbi {
void PublishPointCloud(const ros::Publisher& pub, const PointCloudP& cloud) {
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(cloud, ros_cloud);
  ros_cloud.header.stamp = ros::Time::now();
  pub.publish(ros_cloud);
}

void PublishPointCloud(const ros::Publisher& pub, const PointCloudC& cloud) {
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(cloud, ros_cloud);
  ros_cloud.header.stamp = ros::Time::now();
  pub.publish(ros_cloud);
}
}  // namespace pbi
