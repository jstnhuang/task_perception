#ifndef _TASK_IMITATION_GRASP_PLANNER_H_
#define _TASK_IMITATION_GRASP_PLANNER_H_

#include <string>

#include "Eigen/Eigen"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "task_perception/pr2_gripper_model.h"
#include "task_perception/task_perception_context.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_imitation/grasp_planning_context.h"

namespace pbi {
class GraspFeatures {
 public:
  GraspFeatures();

  int antipodal_grasp_pts;
  int non_antipodal_grasp_pts;
  int antipodal_collisions;
  int non_antipodal_collisions;
  double sq_wrist_distance;
};

class GraspFeatureWeights {
 public:
  GraspFeatureWeights();

  double antipodal_grasp_weight;
  double non_antipodal_grasp_weight;
  double antipodal_collision_weight;
  double non_antipodal_collision_weight;
  double sq_wrist_distance_weight;
};

class GraspEvaluation {
 public:
  GraspEvaluation();
  std::string ToString() const;
  double score() const;

  GraspFeatures features;
  GraspFeatureWeights weights;
};

// PR2 grasp planner that adapts human grasping configurations to robot grasps.
class GraspPlanner {
 public:
  GraspPlanner();

  // Adapts the human grasping configuration to a robot grasping configuration.
  // Output pose is in the camera frame.
  geometry_msgs::Pose Plan(const GraspPlanningContext& context);
  // Same as Plan, but with an initial pose relative to the object as a hint.
  // The initial pose should be given in the camera frame.
  geometry_msgs::Pose Plan(const geometry_msgs::Pose& initial_pose,
                           const GraspPlanningContext& context);

 private:
  // Initialize kGripperMarkers.
  void InitGripperMarkers();
  // Visualize a gripper in the given namespace.
  void VisualizeGripper(const std::string& ns, const geometry_msgs::Pose& pose,
                        const std::string& frame_id);

  // Computes the initial grasp to optimize around.
  geometry_msgs::Pose ComputeInitialGrasp(const Pr2GripperModel& gripper_model,
                                          const GraspPlanningContext& context);
  geometry_msgs::Pose OrientTowardsWrist(const Pr2GripperModel& gripper_model,
                                         const GraspPlanningContext& context);
  geometry_msgs::Pose OptimizeOrientation(const Pr2GripperModel& gripper_model,
                                          const GraspPlanningContext& context);
  GraspEvaluation ScoreGrasp(const Eigen::Affine3d& pose,
                             const Eigen::Vector3d& wrist_pos,
                             const GraspPlanningContext& context);
  geometry_msgs::Pose OptimizePlacement(const geometry_msgs::Pose& gripper_pose,
                                        const GraspPlanningContext& context,
                                        int max_iters);
  void UpdateParams();

  // Internal visualization publishers
  ros::NodeHandle nh_;
  ros::Publisher gripper_pub_;
  ros::Publisher object_pub_;

  // PR2 gripper marker, with wrist_roll_link at the origin and identity
  // orientation.
  visualization_msgs::MarkerArray kGripperMarkers;
  bool debug_;
  GraspFeatureWeights weights_;
};
}  // namespace pbi

#endif  // _TASK_IMITATION_GRASP_PLANNER_H_
