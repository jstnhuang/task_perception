#ifndef _PBI_SHAPE_DETECTION_H_
#define _PBI_SHAPE_DETECTION_H_

#include "task_perception/lazy_object_model.h"

namespace pbi {
// Returns true if the given object is circular.
// By "circular," we mean rotationally symmetric about the model's z-axis.
bool IsCircular(const pcl::PointCloud<pcl::PointXYZ>::Ptr model);
}  // namespace pbi

#endif  // _PBI_SHAPE_DETECTION_H_
