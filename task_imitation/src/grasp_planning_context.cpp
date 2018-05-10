#include "task_imitation/grasp_planning_context.h"

#include "task_perception/pcl_typedefs.h"

using geometry_msgs::Pose;
using std::string;

namespace pbi {
GraspPlanningContext::GraspPlanningContext(
    const Pose& wrist_pose, const string& planning_frame_id,
    const string& object_mesh, const Pose& object_pose,
    const std::vector<Pose>& future_poses,
    const moveit::planning_interface::MoveGroup& move_group,
    ObjectModelCache* model_cache)
    : wrist_pose_(wrist_pose),
      planning_frame_id_(planning_frame_id),
      object_mesh_(object_mesh),
      object_pose_(object_pose),
      future_poses_(future_poses),
      move_group_(move_group),
      lazy_model_(object_mesh, planning_frame_id, object_pose) {
  lazy_model_.set_object_model_cache(model_cache);
}

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

std::vector<Obb> GraspPlanningContext::obstacles() const { return obstacles_; }

const std::vector<geometry_msgs::Pose>& GraspPlanningContext::future_poses()
    const {
  return future_poses_;
}

const moveit::planning_interface::MoveGroup& GraspPlanningContext::move_group()
    const {
  return move_group_;
}

void GraspPlanningContext::AddObstacle(const Obb& obstacle) {
  obstacles_.push_back(obstacle);
}
}  // namespace pbi
