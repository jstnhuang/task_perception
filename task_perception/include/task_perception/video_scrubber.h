#ifndef _PBI_VIDEO_SCRUBBER_H_
#define _PBI_VIDEO_SCRUBBER_H_

#include <vector>

#include "sensor_msgs/Image.h"

namespace pbi {
class VideoScrubber {
 public:
  VideoScrubber();
  void set_images(const std::vector<sensor_msgs::Image>& images);
  void View(const ros::Duration& time, sensor_msgs::Image* image) const;
  void View(const int frame_index, sensor_msgs::Image* image) const;
  int num_images() const;

 private:
  std::vector<sensor_msgs::Image> images_;
};
}  // namespace pbi

#endif  // _PBI_VIDEO_SCRUBBER_H_
