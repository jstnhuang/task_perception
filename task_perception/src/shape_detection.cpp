#include "task_perception/shape_detection.h"

#include <algorithm>

#include "Eigen/Dense"
#include "geometry_msgs/Vector3.h"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"

#include "task_perception/pcl_typedefs.h"

namespace pbi {
bool IsCircular(const PointCloudP::Ptr model) {
  const double kSideLengthRatio = 0.9;

  // Check the ratio of the x/y side lengths
  PointP min_pt;
  PointP max_pt;
  pcl::getMinMax3D(*model, min_pt, max_pt);
  double x_dim = max_pt.x - min_pt.x;
  double y_dim = max_pt.y - min_pt.y;
  double short_side = std::min(x_dim, y_dim);
  double long_side = std::max(x_dim, y_dim);
  if (short_side / long_side < kSideLengthRatio) {
    return false;
  }

  // Rotate the model by 45 degrees and check if the axis-aligned side lengths
  // match the above
  Eigen::AngleAxisf rot(M_PI / 4, Eigen::Vector3f::UnitZ());
  Eigen::Affine3f affine(rot);
  PointCloudP rotated_cloud;
  pcl::transformPointCloud(*model, rotated_cloud, affine);

  pcl::getMinMax3D(rotated_cloud, min_pt, max_pt);
  double rotated_x_dim = max_pt.x - min_pt.x;
  double rotated_y_dim = max_pt.y - min_pt.y;
  double rotated_short = std::min(rotated_x_dim, rotated_y_dim);
  double rotated_long = std::max(rotated_x_dim, rotated_y_dim);
  if (rotated_short / rotated_long < kSideLengthRatio) {
    return false;
  }
  if (rotated_short < short_side) {
    if (rotated_short / short_side < kSideLengthRatio) {
      return false;
    }
  } else {
    if (short_side / rotated_short < kSideLengthRatio) {
      return false;
    }
  }
  if (rotated_long < long_side) {
    if (rotated_long / long_side < kSideLengthRatio) {
      return false;
    }
  } else {
    if (long_side / rotated_long < kSideLengthRatio) {
      return false;
    }
  }
  return true;
}
}  // namespace pbi
