#include "task_perception/pcl_utils.h"

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
  // We do not use ExtractIndices because the build is very fragile.
  // We currently build against dbot, which requires C++ 11. PCL does not build
  // with C++ 11, and random errors occur on startup.
  PointCloudP result;
  result.header = cloud->header;
  for (size_t index_i = 0; index_i < indices->size(); ++index_i) {
    int index = indices->at(index_i);
    result.push_back(cloud->points[index]);
  }
  PublishPointCloud(pub, result);
}
}  // namespace pbi
