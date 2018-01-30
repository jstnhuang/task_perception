#ifndef _PBI_GRASP_PLANNER_H_
#define _PBI_GRASP_PLANNER_H_

#include <set>
#include <string>

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_perception/pr2_gripper_model.h"
#include "task_perception/task_perception_context.h"

namespace pbi {
// PR2 grasp planner that adapts human grasping configurations to robot grasps.
class GraspPlanner {
 public:
  GraspPlanner();

  // Adapts the human grasping configuration to a robot grasping configuration.
  // Output pose is in the camera frame.
  void Plan(const std::string& left_or_right, const std::string& object_name,
            TaskPerceptionContext* context, geometry_msgs::Pose* pose);

 private:
  // Initialize kGripperMarkers.
  void InitGripperMarkers();
  // Visualize a gripper in the given namespace.
  void VisualizeGripper(const std::string& ns, const geometry_msgs::Pose& pose,
                        const std::string& frame_id);

  // Computes the initial grasp to optimize around.
  void ComputeInitialGrasp(const Pr2GripperModel& gripper_model,
                           const std::string& object_name,
                           TaskPerceptionContext* context,
                           geometry_msgs::Pose* initial_pose);
  void OrientTowardsWrist(const Pr2GripperModel& gripper_model,
                          const geometry_msgs::Pose& wrist_pose,
                          TaskPerceptionContext* context,
                          geometry_msgs::Pose* next_pose);
  void OptimizeOrientation(const Pr2GripperModel& gripper_model,
                           const std::string& object_name,
                           TaskPerceptionContext* context,
                           geometry_msgs::Pose* next_pose);
  void OptimizePlacement(const Pr2GripperModel& gripper_model,
                         const std::string& object_name,
                         TaskPerceptionContext* context,
                         geometry_msgs::Pose* next_pose);

  // Internal visualization publishers
  ros::NodeHandle nh_;
  ros::Publisher gripper_pub_;
  ros::Publisher cloud_pub_;

  // PR2 gripper marker, with wrist_roll_link at the origin and identity
  // orientation.
  visualization_msgs::MarkerArray kGripperMarkers;
  const bool kDebug_;
};
}  // namespace pbi

#endif  // _PBI_GRASP_PLANNER_H_
