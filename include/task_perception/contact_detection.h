#ifndef _PBI_CONTACT_DETECTION_H_
#define _PBI_CONTACT_DETECTION_H_

#include <map>
#include <string>

#include "pcl/kdtree/kdtree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/DemoState.h"

#include "task_perception/skeleton_services.h"

namespace pbi {
// Lazily computes context for a single frame of contact detection.
class ContactDetectionContext {
 public:
  ContactDetectionContext(
      pbi::SkeletonServices& skel_services, ros::ServiceClient& predict_hands,
      const task_perception_msgs::DemoState& current_state,
      const task_perception_msgs::DemoState& prev_state,
      const sensor_msgs::Image& color_image,
      const sensor_msgs::Image& depth_image,
      const sensor_msgs::CameraInfo& camera_info,
      std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>*
          object_models);
  bool LoadParams();
  const geometry_msgs::Pose& GetLeftWristPose();
  const geometry_msgs::Pose& GetRightWristPose();
  const std::vector<task_perception_msgs::ObjectState>& GetPrevObjects() const;
  const std::vector<task_perception_msgs::ObjectState>& GetCurrentObjects()
      const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetObjectModel(
      const std::string& mesh_name);
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetObjectCloud(const std::string& name);
  pcl::KdTree<pcl::PointXYZ>::Ptr GetObjectTree(const std::string& name);
  bool GetCurrentObject(const std::string& name,
                        task_perception_msgs::ObjectState* object);
  bool GetPreviousObject(const std::string& name,
                         task_perception_msgs::ObjectState* prev_obj);
  ros::Time GetPreviousTime();
  ros::Time GetCurrentTime();
  pcl::PointCloud<pcl::PointXYZ>::Ptr HandCloud();
  pcl::KdTree<pcl::PointXYZ>::Ptr HandTree();

  // Params
  bool kDebug;
  float kCloseToWristDistance;
  float kMovingObjectDistance;
  float kTouchingObjectDistance;
  int kTouchingObjectPoints;

 private:
  void GetWristPoses();
  // Must be called before using [current,prev]_objects_
  void IndexObjects();
  pcl::PointCloud<pcl::PointXYZ>::Ptr LoadModel(const std::string& mesh_path);

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

  const std::string kPackagePath_;
  bool are_objects_indexed_;
  std::map<std::string, task_perception_msgs::ObjectState> current_objects_;
  std::map<std::string, task_perception_msgs::ObjectState> prev_objects_;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>* object_models_;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> object_clouds_;
  std::map<std::string, pcl::KdTree<pcl::PointXYZ>::Ptr> object_trees_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr hand_cloud_;
  pcl::KdTree<pcl::PointXYZ>::Ptr hand_tree_;
};

class ContactDetection {
 public:
  ContactDetection(const pbi::SkeletonServices& skel_services,
                   const ros::ServiceClient& predict_hands,
                   std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>*
                       object_models);

  void Predict(const task_perception_msgs::DemoState& current_state,
               const task_perception_msgs::DemoState& prev_state,
               const sensor_msgs::Image& color_image,
               const sensor_msgs::Image& depth_image,
               const sensor_msgs::CameraInfo& camera_info,
               task_perception_msgs::HandState* left_hand,
               task_perception_msgs::HandState* right_hand);

 private:
  // Predict the state of a hand given its previous state and other context.
  void PredictHandState(const task_perception_msgs::HandState& prev_state,
                        const std::string& left_or_right,
                        ContactDetectionContext* context,
                        task_perception_msgs::HandState* hand_state);

  // Check if we should transition from NONE -> GRASPING.
  void CheckGrasp(const task_perception_msgs::HandState& prev_state,
                  const std::string& left_or_right,
                  ContactDetectionContext* context,
                  task_perception_msgs::HandState* hand_state);

  // Check if we should transition from [GRASPING, PUSHING] -> NONE.
  void CheckRelease(const task_perception_msgs::HandState& prev_state,
                    const std::string& left_or_right,
                    ContactDetectionContext* context,
                    task_perception_msgs::HandState* hand_state);

  bool IsObjectCurrentlyCloseToWrist(const geometry_msgs::Pose& wrist,
                                     const std::string& object_name,
                                     ContactDetectionContext* context) const;

  bool IsObjectMoving(const task_perception_msgs::ObjectState& object,
                      ContactDetectionContext* context) const;

  int NumHandPointsOnObject(const task_perception_msgs::ObjectState& object,
                            ContactDetectionContext* context) const;

  void PublishWristPoses(const geometry_msgs::Pose& left,
                         const geometry_msgs::Pose& right,
                         const std::string& frame_id);

  pbi::SkeletonServices skel_services_;
  ros::ServiceClient predict_hands_;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>* object_models_;

  ros::NodeHandle nh_;
  // Internal visualizations for debugging purposes
  ros::Publisher viz_;
  ros::Publisher obj_viz_;
  ros::Publisher hand_viz_;
};

// Replace path/to/file.obj to path/to/file.pcd.
std::string ReplaceObjWithPcd(const std::string& path);
}  // namespace pbi

#endif  // _PBI_CONTACT_DETECTION_H_
