#ifndef _PBI_GRASP_PLANNING_CONTEXT_H_
#define _PBI_GRASP_PLANNING_CONTEXT_H_

#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "moveit/move_group_interface/move_group.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception/object_model_cache.h"

#include "task_imitation/obb.h"

namespace pbi {
// Stores key perceptual information necessary for doing local grasp planning
// for imitation.
class GraspPlanningContext {
 public:
  GraspPlanningContext(const geometry_msgs::Pose& wrist_pose,
                       const std::string& planning_frame_id,
                       const std::string& object_mesh,
                       const geometry_msgs::Pose& object_pose,
                       const std::vector<geometry_msgs::Pose>& future_poses,
                       const moveit::planning_interface::MoveGroup& move_group,
                       ObjectModelCache* model_cache);
  geometry_msgs::Pose wrist_pose() const;
  std::string planning_frame_id() const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud() const;
  pcl::PointCloud<pcl::PointNormal>::Ptr object_cloud_with_normals() const;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr object_tree() const;
  std::vector<Obb> obstacles() const;
  const std::vector<geometry_msgs::Pose>& future_poses() const;
  const moveit::planning_interface::MoveGroup& move_group() const;

  void AddObstacle(const Obb& obstacle);

 private:
  geometry_msgs::Pose wrist_pose_;
  std::string planning_frame_id_;
  std::string object_mesh_;
  geometry_msgs::Pose object_pose_;
  std::vector<geometry_msgs::Pose> future_poses_;
  const moveit::planning_interface::MoveGroup& move_group_;

  mutable ObjectModelCache* model_cache_;
  LazyObjectModel lazy_model_;
  std::vector<Obb> obstacles_;
};
}  // namespace pbi

#endif  // _PBI_GRASP_PLANNING_CONTEXT_H_
