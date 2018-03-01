#ifndef _TASK_IMITATION_GRASP_PLANNER_H_
#define _TASK_IMITATION_GRASP_PLANNER_H_

#include <set>
#include <string>

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_perception/pr2_gripper_model.h"
#include "task_perception/task_perception_context.h"

namespace pbi {
class GraspFeatures {
 public:
  GraspFeatures();

  int antipodal_grasp_pts;
  int non_antipodal_grasp_pts;
  int antipodal_collisions;
  int non_antipodal_collisions;
  double sq_wrist_distance;
  double sq_prev_distance;
};

class GraspFeatureWeights {
 public:
  GraspFeatureWeights();

  double antipodal_grasp_weight;
  double non_antipodal_grasp_weight;
  double antipodal_collision_weight;
  double non_antipodal_collision_weight;
  double sq_wrist_distance_weight;
  double sq_prev_distance_weight;
};

class GraspEvaluation {
 public:
  GraspEvaluation();
  std::string ToString() const;
  double score() const;

  GraspFeatures features;
  GraspFeatureWeights weights;
};

// Contains data relevant for grasp planning
struct GraspPlanningContext {
  std::string planning_frame;      // The ID
  geometry_msgs::Pose wrist_pose;  // Human wrist pose in planning frame
  std::string object_name;
  std::string object_mesh;
  geometry_msgs::Pose object_pose;  // Object pose in planning frame
};

// PR2 grasp planner that adapts human grasping configurations to robot grasps.
class GraspPlanner {
 public:
  GraspPlanner();

  // Adapts the human grasping configuration to a robot grasping configuration.
  // Output pose is in the camera frame.
  geometry_msgs::Pose Plan(const std::string& left_or_right,
                           const std::string& object_name,
                           const GraspPlanningContext& context);
  // Same as Plan, but with an initial pose relative to the object as a hint.
  // The initial pose should be given in the camera frame.
  geometry_msgs::Pose Plan(const std::string& left_or_right,
                           const std::string& object_name,
                           const geometry_msgs::Pose& initial_pose,
                           const GraspPlanningContext& context);

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
                           const geometry_msgs::Pose& wrist_pose,
                           const geometry_msgs::Pose& prev_gripper_pose,
                           TaskPerceptionContext* context,
                           geometry_msgs::Pose* next_pose);
  void ScoreGrasp(const Eigen::Affine3d& pose, const std::string& object_name,
                  const Eigen::Vector3d& wrist_pos,
                  const Eigen::Vector3d& prev_gripper_pose,
                  TaskPerceptionContext* context, GraspEvaluation* result);
  void OptimizePlacement(const geometry_msgs::Pose& gripper_pose,
                         const std::string& object_name,
                         TaskPerceptionContext* context, int max_iters,
                         geometry_msgs::Pose* next_pose);
  void UpdateParams();

  void GetPreviousGripperPose(const std::string& left_or_right,
                              TaskPerceptionContext* context,
                              geometry_msgs::Pose* prev_pose);

  // Internal visualization publishers
  ros::NodeHandle nh_;
  ros::Publisher gripper_pub_;

  // PR2 gripper marker, with wrist_roll_link at the origin and identity
  // orientation.
  visualization_msgs::MarkerArray kGripperMarkers;
  bool debug_;
  GraspFeatureWeights weights_;
};
}  // namespace pbi

#endif  // _TASK_IMITATION_GRASP_PLANNER_H_
