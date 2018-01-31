#ifndef _PBI_DEFAULT_SKELETON_H_
#define _PBI_DEFAULT_SKELETON_H_

#include "skin_segmentation_msgs/NerfJointStates.h"

namespace pbi {
// Returns an initial skeleton that works well for the PR2 at full height and
// camera at 45 degrees.
skin_segmentation_msgs::NerfJointStates DefaultSkeleton();
}  // namespace pbi

#endif  // _PBI_DEFAULT_SKELETON_H_
