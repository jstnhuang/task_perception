#ifndef _PBI_PCL_TYPEDEFS_H_
#define _PBI_PCL_TYPEDEFS_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

// Defines convenient typedefs for PCL.
// Do not include this file in other header files.

typedef pcl::PointXYZ PointP;
typedef pcl::PointXYZRGB PointC;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudP;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

#endif  // _PBI_PCL_TYPEDEFS_H_
