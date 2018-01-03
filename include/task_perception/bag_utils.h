#ifndef _PBI_BAG_UTILS_H_
#define _PBI_BAG_UTILS_H_

#include <string>

#include "rosbag/bag.h"
#include "sensor_msgs/CameraInfo.h"

namespace pbi {
// Gets the RGB / depth topic names in a bag file.
//
// The first Image topic that contains "rgb" is assumed to be the RGB image
// topic. The first Image topic that contains "depth" is the depth image topic.
bool GetImageTopics(const rosbag::Bag& bag, std::string* rgb_topic,
                    std::string* depth_topic);

// Gets the first CameraInfo message in a bag file.
bool GetCameraInfo(const rosbag::Bag& bag,
                   sensor_msgs::CameraInfo* camera_info);

// Gets the duration of the recording.
//
// Duration of the recording is measured between the first and last images.
double GetBagDuration(const rosbag::Bag& bag);
}  // namespace pbi

#endif  // _PBI_BAG_UTILS_H_
