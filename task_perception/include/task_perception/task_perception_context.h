#ifndef _PBI_TASK_PERCEPTION_CONTEXT_H_
#define _PBI_TASK_PERCEPTION_CONTEXT_H_

#include <map>
#include <string>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/DemoState.h"

#include "task_perception/lazy_object_model.h"
#include "task_perception/skeleton_services.h"

namespace pbi {
// Lazily computes context for a single frame of a demonstration.
class TaskPerceptionContext {
 public:
  typedef std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>
      ObjectModelCache;
  TaskPerceptionContext(pbi::SkeletonServices& skel_services,
                        ros::ServiceClient& predict_hands,
                        const task_perception_msgs::DemoState& current_state,
                        const task_perception_msgs::DemoState& prev_state,
                        const sensor_msgs::Image& color_image,
                        const sensor_msgs::Image& depth_image,
                        const sensor_msgs::CameraInfo& camera_info,
                        ObjectModelCache* object_models);
  bool LoadParams();
  const geometry_msgs::Pose& GetLeftWristPose();
  const geometry_msgs::Pose& GetRightWristPose();
  const std::vector<task_perception_msgs::ObjectState>& GetPrevObjects() const;
  const std::vector<task_perception_msgs::ObjectState>& GetCurrentObjects()
      const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetObjectModel(
      const std::string& mesh_name);
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetObjectCloud(const std::string& name);
  pcl::PointCloud<pcl::PointNormal>::Ptr GetObjectCloudWithNormals(
      const std::string& name);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr GetObjectTree(
      const std::string& name);
  bool GetCurrentObject(const std::string& name,
                        task_perception_msgs::ObjectState* object);
  bool GetPreviousObject(const std::string& name,
                         task_perception_msgs::ObjectState* prev_obj);
  ros::Time GetPreviousTime();
  ros::Time GetCurrentTime();
  pcl::PointCloud<pcl::PointXYZ>::Ptr BothHandsCloud();
  pcl::IndicesPtr LeftHandIndices();
  pcl::IndicesPtr RightHandIndices();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr LeftHandTree();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr RightHandTree();
  const task_perception_msgs::DemoState& current_state() const;
  const task_perception_msgs::DemoState& prev_state() const;
  const sensor_msgs::CameraInfo& camera_info() const;

  // Params
  bool kDebug;
  float kCloseToWristDistance;
  float kMovingObjectDistance;
  float kTouchingObjectDistance;
  float kTouchingReleasedObjectDistance;
  int kTouchingObjectPoints;
  int kTouchingReleasedObjectPoints;
  float kPartOfHandDistance;

 private:
  void GetWristPoses();
  // Must be called before using [current,prev]_objects_
  void IndexObjects();
  void ComputeHandClouds();
  void ComputeLeftRightHands();

  pbi::SkeletonServices& skel_services_;
  ros::ServiceClient& predict_hands_;
  const task_perception_msgs::DemoState& current_state_;
  const task_perception_msgs::DemoState& prev_state_;
  const sensor_msgs::Image& color_image_;
  const sensor_msgs::Image& depth_image_;
  const sensor_msgs::CameraInfo& camera_info_;

  // Cached contextual information
  // Wrist pose
  bool have_wrist_poses_;
  geometry_msgs::Pose left_wrist_pose_;
  geometry_msgs::Pose right_wrist_pose_;

  // Object models
  bool are_objects_indexed_;
  std::map<std::string, task_perception_msgs::ObjectState> current_objects_;
  std::map<std::string, task_perception_msgs::ObjectState> prev_objects_;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>* object_models_;
  std::map<std::string, LazyObjectModel> lazy_objects_;

  // Hand segmentation
  pcl::PointCloud<pcl::PointXYZ>::Ptr both_hands_cloud_;
  pcl::IndicesPtr left_hand_indices_;
  pcl::IndicesPtr right_hand_indices_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr left_hand_tree_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr right_hand_tree_;
};
}  // namespace pbi

#endif  // _PBI_TASK_PERCEPTION_CONTEXT_H_
