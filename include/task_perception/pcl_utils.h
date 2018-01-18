#ifndef _PBI_PCL_UTILS_H_
#define _PBI_PCL_UTILS_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"

namespace pbi {
void PublishPointCloud(const ros::Publisher& pub,
                       const pcl::PointCloud<pcl::PointXYZ>& cloud);
void PublishPointCloud(const ros::Publisher& pub,
                       const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
}  // namespace pbi

#endif  // _PBI_PCL_UTILS_H_
