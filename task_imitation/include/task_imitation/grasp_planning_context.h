#ifndef _PBI_GRASP_PLANNING_CONTEXT_H_
#define _PBI_GRASP_PLANNING_CONTEXT_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "task_perception/lazy_object_model.h"

namespace pbi {
// Stores key perceptual information necessary for doing local grasp planning
// for imitation.
class GraspPlanningContext {
 public:
  GraspPlanningContext(const geometry_msgs::Pose& wrist_pose,
                       const std::string& planning_frame_id,
                       const std::string& object_name,
                       const std::string& object_mesh,
                       const geometry_msgs::Pose& object_pose);
  geometry_msgs::Pose wrist_pose() const;
  std::string planning_frame_id() const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud() const;
  pcl::PointCloud<pcl::PointNormal>::Ptr object_cloud_with_normals() const;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr object_tree() const;

 private:
  geometry_msgs::Pose wrist_pose_;
  std::string planning_frame_id_;
  std::string object_name_;
  std::string object_mesh_;
  geometry_msgs::Pose object_pose_;
  LazyObjectModel lazy_model_;
};
}  // namespace pbi

#endif  // _PBI_GRASP_PLANNING_CONTEXT_H_
