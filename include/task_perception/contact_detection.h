#ifndef _PBI_CONTACT_DETECTION_H_
#define _PBI_CONTACT_DETECTION_H_

#include <map>
#include <string>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/DemoState.h"

#include "task_perception/skeleton_services.h"

namespace pbi {
class ContactDetection {
 public:
  ContactDetection(const pbi::SkeletonServices& skel_services,
                   const ros::ServiceClient& predict_hands);

  void Predict(const task_perception_msgs::DemoState& current_state,
               const task_perception_msgs::DemoState& prev_state,
               const sensor_msgs::Image& color_image,
               const sensor_msgs::Image& depth_image,
               const sensor_msgs::CameraInfo& camera_info);

 private:
  void PublishWristPoses(const geometry_msgs::Pose& left,
                         const geometry_msgs::Pose& right,
                         const std::string& frame_id);
  pcl::PointCloud<pcl::PointXYZ>::Ptr LoadModel(
      const std::string& mesh_name_obj);
  pcl::PointCloud<pcl::PointXYZ>::Ptr HandPointCloud(
      const sensor_msgs::Image& hands, const sensor_msgs::Image& depth,
      const sensor_msgs::CameraInfo& camera_info);

  pbi::SkeletonServices skel_services_;
  ros::ServiceClient predict_hands_;

  bool debug_;
  ros::NodeHandle nh_;
  // Internal visualizations for debugging purposes
  ros::Publisher viz_;
  ros::Publisher obj_viz_;

  const std::string package_dir_;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> model_cache_;
};
}  // namespace pbi

#endif  // _PBI_CONTACT_DETECTION_H_
