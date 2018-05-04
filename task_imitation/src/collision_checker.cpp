#include "task_imitation/collision_checker.h"

#include <limits.h>

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
CollisionChecker::CollisionChecker(const std::string& planning_frame)
    : planning_frame_(planning_frame), model_cache_() {}

std::string CollisionChecker::Check(
    const task_perception_msgs::ObjectState& object,
    const std::vector<task_perception_msgs::ObjectState>& other_objects) const {
  const double kInflationSize =
      rapid::GetDoubleParamOrThrow("task_imitation/object_inflation_size");
  LazyObjectModel held_obj_model(object.mesh_name, planning_frame_,
                                 object.pose);
  held_obj_model.set_object_model_cache(&model_cache_);
  const Pose& held_obj_pose = held_obj_model.pose();
  Eigen::Vector3d held_obj_vec = rapid::AsVector3d(held_obj_pose.position);
  geometry_msgs::Vector3 held_obj_scale =
      InflateScale(held_obj_model.scale(), kInflationSize);
  double closest_sq_distance = std::numeric_limits<double>::max();
  std::string closest_collidee("");
  BOOST_FOREACH (const msgs::ObjectState& other, other_objects) {
    if (other.name == object.name) {
      continue;
    }
    LazyObjectModel other_model(other.mesh_name, planning_frame_, other.pose);
    other_model.set_object_model_cache(&model_cache_);
    if (rapid::AreObbsInCollision(
            held_obj_pose, held_obj_scale, other_model.pose(),
            InflateScale(other_model.scale(), kInflationSize))) {
      Eigen::Vector3d other_vec =
          rapid::AsVector3d(other_model.pose().position);
      double sq_distance = (held_obj_vec - other_vec).squaredNorm();
      if (sq_distance < closest_sq_distance) {
        closest_sq_distance = sq_distance;
        closest_collidee = other.name;
      }
    }
  }
  return closest_collidee;
}

bool CollisionChecker::Check(const msgs::ObjectState& obj1,
                             const msgs::ObjectState& obj2) const {
  LazyObjectModel obj1_model(obj1.mesh_name, planning_frame_, obj1.pose);
  LazyObjectModel obj2_model(obj2.mesh_name, planning_frame_, obj2.pose);
  obj1_model.set_object_model_cache(&model_cache_);
  obj2_model.set_object_model_cache(&model_cache_);
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
