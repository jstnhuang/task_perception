#ifndef _TASK_IMITATION_GRASP_PLANNER_H_
#define _TASK_IMITATION_GRASP_PLANNER_H_

#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"
#include "task_perception/pr2_gripper_model.h"
#include "task_utils/pr2_gripper_viz.h"
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
  int num_obstacle_collisions;
  double future_pose_ratio;
};

class GraspFeatureWeights {
 public:
  GraspFeatureWeights();

  double antipodal_grasp_weight;
  double non_antipodal_grasp_weight;
  double antipodal_collision_weight;
  double non_antipodal_collision_weight;
  double sq_wrist_distance_weight;
  double obstacle_collision_weight;
  double future_pose_weight;
};

class GraspEvaluation {
 public:
  GraspEvaluation();
  std::string ToString() const;
  double score() const;

  GraspFeatures features;
  GraspFeatureWeights weights;
};

class ScoredGrasp {
 public:
  ScoredGrasp();
  inline bool IsValid();

  double score;
  GraspEvaluation eval;
  geometry_msgs::Pose pose;
};

// PR2 grasp planner that adapts human grasping configurations to robot grasps.
class GraspPlanner {
 public:
  GraspPlanner(const Pr2GripperViz& gripper_viz);

  // Adapts the human grasping configuration to a robot grasping configuration.
  // Output pose is in the camera frame.
  geometry_msgs::Pose Plan(const GraspPlanningContext& context);
  // Same as Plan, but with an initial pose relative to the object as a hint.
  // The initial pose should be given in the camera frame.
  geometry_msgs::Pose Plan(const geometry_msgs::Pose& initial_pose,
                           const GraspPlanningContext& context);

 private:
  // Visualize a gripper in the given namespace.
  void VisualizeGripper(const std::string& ns, const geometry_msgs::Pose& pose,
                        const std::string& frame_id);

  // Computes the initial grasp to optimize around.
  geometry_msgs::Pose ComputeInitialGrasp(const Pr2GripperModel& gripper_model,
                                          const GraspPlanningContext& context);

  // GRASPING 2.0
  // Returns N closest points on object to the grasp center.
  std::vector<int> SampleObject(const GraspPlanningContext& context,
                                const double leaf_size);

  // Publishes a visualization of the normal vector of an object point.
  void VisualizePointNormal(const GraspPlanningContext& context,
                            const int index);

  // Move the grasp such that the given point is in the center of the grasp.
  geometry_msgs::Pose CenterGraspOnPoint(const Pr2GripperModel& gripper_model,
                                         const GraspPlanningContext& context,
                                         const int index);

  // Finds a grasp centered on a given point and with the the y-axis of the
  // gripper aligned with the normal of the point.
  geometry_msgs::Pose AlignGraspWithPoint(const Pr2GripperModel& gripper_model,
                                          const GraspPlanningContext& context,
                                          const int index);

  geometry_msgs::Pose PitchToWrist(const Pr2GripperModel& gripper_model,
                                   const GraspPlanningContext& context);

  // Try a number of different pitch angles and choose the best one.
  // The best pitch meets the following constraints:
  // 1) There are object points in the the grasp region
  // 2) The gripper is not in collision with any obstacles
  //
  // It also optimizes over ScoreGrasp.
  ScoredGrasp OptimizePitch(const Pr2GripperModel& gripper_model,
                            const double obj_width,
                            const GraspPlanningContext& context);

  ScoredGrasp EscapeCollision(const Pr2GripperModel& gripper_model,
                              const GraspPlanningContext& context);

  geometry_msgs::Pose OrientTowardsWrist(const Pr2GripperModel& gripper_model,
                                         const GraspPlanningContext& context);
  geometry_msgs::Pose OptimizeOrientation(const Pr2GripperModel& gripper_model,
                                          const GraspPlanningContext& context);
  GraspEvaluation ScoreGrasp(const Pr2GripperModel& model,
                             const Eigen::Vector3d& wrist_pos,
                             const GraspPlanningContext& context);

  // Translates the gripper pose to collect points in the grasp region.
  geometry_msgs::Pose OptimizePlacement(const geometry_msgs::Pose& gripper_pose,
                                        const GraspPlanningContext& context,
                                        int max_iters);

  // Like OptimizePlacement, but maximizes the margin between the object points
  // in the grasp region and the gripper.
  geometry_msgs::Pose MaximizeMargin(const geometry_msgs::Pose& gripper_pose,
                                     const GraspPlanningContext& context);

  // Returns the number of future poses that can be reached with the given
  // grasp.
  int EvaluateFuturePoses(const Pr2GripperModel& model,
                          const GraspPlanningContext& context);

  void UpdateParams();

  double ComputeObjWidthInGraspRegion(const geometry_msgs::Pose& gripper_pose,
                                      const GraspPlanningContext& context);

  // Internal visualization publishers
  ros::NodeHandle nh_;
  ros::Publisher gripper_pub_;
  ros::Publisher object_pub_;

  // PR2 gripper marker, with wrist_roll_link at the origin and identity
  // orientation.
  const Pr2GripperViz& gripper_viz_;
  bool debug_;

  // Weights and params
  GraspFeatureWeights weights_;
  double kAntipodalCos;

  ros::Publisher debug_cloud_pub_;
  std::vector<GraspEvaluation> evals_;
  std::vector<int> labels_;
};

int NumObstacleCollisions(const Pr2GripperModel& gripper,
                          const GraspPlanningContext& context);
bool IsPalmCollidingWithAllObstacles(const Pr2GripperModel& gripper,
                                     const GraspPlanningContext& context);
int NumCollisions(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr obj_in_gripper);
int NumPointsInGraspRegion(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr obj_in_gripper);

// Checks if there's a plan from the pre-grasp pose to the pose.
// pose is given in the planning frame.
bool IsGraspReachable(moveit::planning_interface::MoveGroup& move_group,
                      const geometry_msgs::Pose& pose);
}  // namespace pbi

#endif  // _TASK_IMITATION_GRASP_PLANNER_H_
