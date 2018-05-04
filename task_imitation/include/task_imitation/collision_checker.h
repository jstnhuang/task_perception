#ifndef _PBI_COLLISION_CHECKER_H_
#define _PBI_COLLISION_CHECKER_H_

#include <string>
#include <vector>

#include "geometry_msgs/Vector3.h"
#include "task_perception/object_model_cache.h"
#include "task_perception_msgs/ObjectState.h"

namespace pbi {
// Collision checking class with an object model cache.
class CollisionChecker {
 public:
  CollisionChecker(const std::string& planning_frame,
                   ObjectModelCache* model_cache);
  std::string Check(const task_perception_msgs::ObjectState& object,
                    const std::vector<task_perception_msgs::ObjectState>&
                        other_objects) const;
  bool Check(const task_perception_msgs::ObjectState& obj1,
             const task_perception_msgs::ObjectState& obj2) const;

 private:
  std::string planning_frame_;
  mutable ObjectModelCache* model_cache_;
};

// Add a constant to a vector3.
geometry_msgs::Vector3 InflateScale(const geometry_msgs::Vector3& scale,
                                    double distance);
}  // namespace pbi

#endif  // _PBI_COLLISION_CHECKER_H_
