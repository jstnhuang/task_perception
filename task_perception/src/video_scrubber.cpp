#include "task_perception/video_scrubber.h"

#include <algorithm>
#include <vector>

#include "sensor_msgs/Image.h"

namespace {
bool EarlierThan(const sensor_msgs::Image& a, const sensor_msgs::Image& b) {
  return a.header.stamp < b.header.stamp;
}
}

namespace pbi {
VideoScrubber::VideoScrubber() : images_() {}

void VideoScrubber::set_images(const std::vector<sensor_msgs::Image>& images) {
  images_ = images;
}

void VideoScrubber::View(const ros::Duration& time,
                         sensor_msgs::Image* image) const {
  if (images_.size() == 0) {
    return;
  }
  sensor_msgs::Image value;
  value.header.stamp = images_[0].header.stamp + time;
  std::vector<sensor_msgs::Image>::const_iterator it =
      std::lower_bound(images_.begin(), images_.end(), value, EarlierThan);
  if (it != images_.begin()) {
    --it;
  }
  *image = *it;
}

void VideoScrubber::View(const int frame_index,
                         sensor_msgs::Image* image) const {
  if (frame_index < 0 ||
      static_cast<std::vector<sensor_msgs::Image>::size_type>(frame_index) >=
          images_.size()) {
    return;
  }
  *image = images_[frame_index];
}

int VideoScrubber::num_images() const { return images_.size(); }
}  // namespace pbi
