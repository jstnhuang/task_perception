#ifndef _PBI_PCL_UTILS_H_
#define _PBI_PCL_UTILS_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"

namespace pbi {
void PublishPointCloud(const ros::Publisher& pub,
                       const pcl::PointCloud<pcl::PointXYZ>& cloud);

void PublishPointCloud(const ros::Publisher& pub,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const boost::shared_ptr<std::vector<int> >& indices);

void PublishPointCloud(const ros::Publisher& pub,
                       const pcl::PointCloud<pcl::PointXYZRGB>& cloud);

void PublishPointCloud(const ros::Publisher& pub,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                       const boost::shared_ptr<std::vector<int> >& indices);
}  // namespace pbi

#endif  // _PBI_PCL_UTILS_H_
