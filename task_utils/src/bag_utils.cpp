#include "task_utils/bag_utils.h"

#include <string>
#include <vector>

#include "boost/algorithm/string.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/CameraInfo.h"

namespace pbi {
bool GetImageTopics(const rosbag::Bag& bag, std::string* rgb_topic,
                    std::string* depth_topic) {
  rosbag::View view(bag, rosbag::TypeQuery("sensor_msgs/Image"));
  bool found_rgb = false;
  bool found_depth = false;
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    const std::string& topic = it->getTopic();
    if (topic.find("rgb") != std::string::npos) {
      *rgb_topic = topic;
      found_rgb = true;
      continue;
    }
    if (topic.find("depth") != std::string::npos) {
      *depth_topic = topic;
      found_depth = true;
    }
    if (found_rgb && found_depth) {
      return true;
    }
  }
  return false;
}

bool GetCameraInfo(const rosbag::Bag& bag,
                   sensor_msgs::CameraInfo* camera_info) {
  rosbag::View view(bag, rosbag::TypeQuery("sensor_msgs/CameraInfo"));
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    *camera_info = *it->instantiate<sensor_msgs::CameraInfo>();
    return true;
  }
  return false;
}

int GetNumMessagesOnTopic(const rosbag::Bag& bag, const std::string& topic) {
  std::vector<std::string> topics;
  topics.push_back(topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  return view.size();
}

std::string GetNameFromBagPath(const std::string& bag_path) {
  std::vector<std::string> parts;
  boost::split(parts, bag_path, boost::is_any_of("/"));
  std::string last_bag_part(parts[parts.size() - 1]);
  std::string bag_name(last_bag_part.substr(0, last_bag_part.size() - 4));
  return bag_name;
}
}  // namespace pbi
