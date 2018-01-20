#include "task_perception/pcl_utils.h"

#include "pcl/filters/extract_indices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
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

void PublishPointCloud(const ros::Publisher& pub, const PointCloudP::Ptr& cloud,
                       const pcl::IndicesPtr& indices) {
  pcl::ExtractIndices<PointP> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  PointCloudP result;
  extract.filter(result);
  PublishPointCloud(pub, result);
}

void PublishPointCloud(const ros::Publisher& pub, const PointCloudC& cloud) {
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(cloud, ros_cloud);
  ros_cloud.header.stamp = ros::Time::now();
  pub.publish(ros_cloud);
}

void PublishPointCloud(const ros::Publisher& pub, const PointCloudC::Ptr& cloud,
                       const pcl::IndicesPtr& indices) {
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  PointCloudC result;
  extract.filter(result);
  PublishPointCloud(pub, result);
}
}  // namespace pbi
