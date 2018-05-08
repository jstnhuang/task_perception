#include "task_imitation/collision_checker.h"

#include <limits.h>
#include <algorithm>
#include <utility>

#include "Eigen/Dense"
#include "boost/foreach.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "rapid_collision/collision_checks.h"
#include "rapid_ros/params.h"
#include "rapid_utils/vector3.hpp"

#include "task_perception/lazy_object_model.h"

namespace msgs = task_perception_msgs;
using geometry_msgs::Pose;

namespace pbi {
CollisionChecker::CollisionChecker(const std::string& planning_frame,
                                   ObjectModelCache* model_cache)
    : planning_frame_(planning_frame), model_cache_(model_cache) {}

std::vector<std::string> CollisionChecker::Check(
    const task_perception_msgs::ObjectState& object,
    const std::vector<task_perception_msgs::ObjectState>& other_objects) const {
  const double kInflationSize =
      rapid::GetDoubleParamOrThrow("task_imitation/object_inflation_size");
  LazyObjectModel held_obj_model(object.mesh_name, planning_frame_,
                                 object.pose);
  held_obj_model.set_object_model_cache(model_cache_);
  const Pose& held_obj_pose = held_obj_model.pose();
  Eigen::Vector3d held_obj_vec = rapid::AsVector3d(held_obj_pose.position);
  geometry_msgs::Vector3 held_obj_scale =
      InflateScale(held_obj_model.scale(), kInflationSize);
  std::vector<std::pair<double, std::string> > scored_collidees;
  BOOST_FOREACH (const msgs::ObjectState& other, other_objects) {
    if (other.name == object.name) {
      continue;
    }
    LazyObjectModel other_model(other.mesh_name, planning_frame_, other.pose);
    other_model.set_object_model_cache(model_cache_);
    if (rapid::AreObbsInCollision(
            held_obj_pose, held_obj_scale, other_model.pose(),
            InflateScale(other_model.scale(), kInflationSize))) {
      Eigen::Vector3d other_vec =
          rapid::AsVector3d(other_model.pose().position);
      double sq_distance = (held_obj_vec - other_vec).squaredNorm();
      scored_collidees.push_back(
          std::make_pair<double, std::string>(sq_distance, other.name));
    }
  }
  std::sort(scored_collidees.begin(), scored_collidees.end());
  std::vector<std::string> collidees(scored_collidees.size());
  for (size_t i = 0; i < scored_collidees.size(); ++i) {
    collidees[i] = scored_collidees[i].second;
  }
  return collidees;
}

bool CollisionChecker::Check(const msgs::ObjectState& obj1,
                             const msgs::ObjectState& obj2) const {
  LazyObjectModel obj1_model(obj1.mesh_name, planning_frame_, obj1.pose);
  obj1_model.set_object_model_cache(model_cache_);
  LazyObjectModel obj2_model(obj2.mesh_name, planning_frame_, obj2.pose);
  obj2_model.set_object_model_cache(model_cache_);
  const double kInflationSize =
      rapid::GetDoubleParamOrThrow("task_imitation/object_inflation_size");

  geometry_msgs::Vector3 obj1_scale =
      InflateScale(obj1_model.scale(), kInflationSize);
  geometry_msgs::Vector3 obj2_scale =
      InflateScale(obj2_model.scale(), kInflationSize);
  return rapid::AreObbsInCollision(obj1_model.pose(), obj1_scale,
                                   obj2_model.pose(), obj2_scale);
}

geometry_msgs::Vector3 InflateScale(const geometry_msgs::Vector3& scale,
                                    double distance) {
  geometry_msgs::Vector3 inflated = scale;
  inflated.x += distance;
  inflated.y += distance;
  inflated.z += distance;
  return inflated;
}
}  // namespace pbi
