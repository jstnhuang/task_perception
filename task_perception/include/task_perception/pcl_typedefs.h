#ifndef _PBI_PCL_TYPEDEFS_H_
#define _PBI_PCL_TYPEDEFS_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"

// Defines convenient typedefs for PCL.
// Do not include this file in other header files.

typedef pcl::PointXYZ PointP;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointNormal PointN;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudP;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
typedef pcl::search::KdTree<pcl::PointXYZ> KdTreeP;

#endif  // _PBI_PCL_TYPEDEFS_H_
