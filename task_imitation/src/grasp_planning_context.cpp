#include "task_imitation/grasp_planning_context.h"

#include <string>

#include "geometry_msgs/Pose.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "task_perception/pcl_typedefs.h"

using geometry_msgs::Pose;
using std::string;

namespace pbi {
GraspPlanningContext::GraspPlanningContext(const Pose& wrist_pose,
                                           const string& planning_frame_id,
                                           const string& object_name,
                                           const string& object_mesh,
                                           const Pose& object_pose)
    : wrist_pose_(wrist_pose),
      planning_frame_id_(planning_frame_id),
      object_name_(object_name),
      object_mesh_(object_mesh),
      object_pose_(object_pose),
      lazy_model_(object_mesh, planning_frame_id, object_pose) {}

Pose GraspPlanningContext::wrist_pose() const { return wrist_pose_; }

std::string GraspPlanningContext::planning_frame_id() const {
  return planning_frame_id_;
}

PointCloudP::Ptr GraspPlanningContext::object_cloud() const {
  return lazy_model_.GetObjectCloud();
}

PointCloudN::Ptr GraspPlanningContext::object_cloud_with_normals() const {
  return lazy_model_.GetObjectCloudWithNormals();
}

KdTreeP::Ptr GraspPlanningContext::object_tree() const {
  return lazy_model_.GetObjectTree();
}
}  // namespace pbi
