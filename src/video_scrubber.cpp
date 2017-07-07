#include "task_perception/video_scrubber.h"

#include <vector>

#include "sensor_msgs/Image.h"

namespace pbi {
VideoScrubber::VideoScrubber() : images_() {}

void VideoScrubber::set_images(const std::vector<sensor_msgs::Image>& images) {
  images_ = images;
}

void VideoScrubber::View(const ros::Duration& time, sensor_msgs::Image* image) {
}
}  // namespace pbi
