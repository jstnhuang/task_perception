#ifndef _PBI_PROGRAM_GENERATOR_H_
#define _PBI_PROGRAM_GENERATOR_H_

#include <map>
#include <string>

#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception/object_model_cache.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/Step.h"
#include "task_utils/pr2_gripper_viz.h"

#include "task_imitation/collision_checker.h"
#include "task_imitation/obb.h"
#include "task_imitation/program_segment.h"
#include "task_imitation/typed_pose.h"

namespace pbi {
// Generates an executable robot program given a sequence of states extracted
// from a demonstration.
class ProgramGenerator {
 public:
  typedef std::map<std::string, task_perception_msgs::ObjectState>
      ObjectStateIndex;

  ProgramGenerator(moveit::planning_interface::MoveGroup& left_group,
                   moveit::planning_interface::MoveGroup& right_group,
                   ObjectModelCache* model_cache,
                   const Pr2GripperViz& gripper_viz);
  task_perception_msgs::Program Generate(
      const std::vector<task_perception_msgs::DemoState>& demo_states,
      const ObjectStateIndex& initial_objects, const Obb& table);

 private:
  std::vector<ProgramSegment> Segment(
      const std::vector<task_perception_msgs::DemoState>& demo_states);

  // First, process as much as possible without knowing the grasp. This means
  // computing and storing the object trajectories in the ee_trajectory field of
  // each Step. Later, PlanGrasps will plan the grasps and
  // ParameterizeStepsWithGrasps will update all the steps with the planned
  // grasps.
  void ProcessSegmentWithoutGrasps(
      const std::vector<ProgramSegment>& segments, const size_t index,
      const ObjectStateIndex& initial_runtime_objects,
      const ObjectStateIndex& initial_demo_objects);

  // index is the index of the grasp step to plan. All steps are passed in so
  // that the grasp planner can plan a grasp for future steps.
  task_perception_msgs::Step PlanGrasp(
      const std::vector<task_perception_msgs::Step>& steps, const size_t index,
      const geometry_msgs::Pose& wrist_in_obj,
      const std::vector<Obb>& obstacles);

  // Populate the grasp step with everything except the actual grasp pose.
  void AddGraspStep(const ProgramSegment& segments,
                    const ObjectStateIndex& initial_runtime_objects);
  // Adds an ungrasp step to the program based on the given segment.
  void AddUngraspStep(const ProgramSegment& segment);
  void AddMoveToStep(const ProgramSegment& segment,
                     const ObjectStateIndex& initial_demo_objects,
                     const ObjectStateIndex& initial_runtime_objects);
  task_perception_msgs::Step ParameterizeMoveToWithGrasp(
      const ProgramSegment& segment,
      const task_perception_msgs::Step& move_step,
      const geometry_msgs::Pose& gripper_in_obj);
  void AddTrajectoryStep(const ProgramSegment& segment,
                         const ObjectStateIndex& initial_demo_objects,
                         const ObjectStateIndex& initial_runtime_objects);
  task_perception_msgs::Step ParameterizeTrajectoryWithGrasp(
      const ProgramSegment& segment,
      const task_perception_msgs::Step& traj_step,
      const geometry_msgs::Pose& gripper_in_obj, const Obb& table);

  task_perception_msgs::Program program_;

  // The real-world timestamp of first step we take.
  ros::Time start_time_;

  moveit::planning_interface::MoveGroup& left_group_;
  moveit::planning_interface::MoveGroup& right_group_;
  std::string planning_frame_;

  ObjectModelCache* model_cache_;
  CollisionChecker collision_checker_;
  const Pr2GripperViz& gripper_viz_;
  ros::NodeHandle nh_;
  ros::Publisher gripper_pub_;
};

// Gets the initial states of all the objects at the time of the demonstration.
// NOTE: the poses of these objects are in the camera frame. In contrast, the
// poses of "runtime objects" are given in the robot's planning/base frame.
ProgramGenerator::ObjectStateIndex GetInitialDemoObjects(
    const std::vector<task_perception_msgs::DemoState>& demo_states);

// Gets the pose of a target object. If the object is circular, it will be
// rotated so that its x-axis points in the x direction of the planning
// frame.
// geometry_msgs::Pose GetTargetPose(const LazyObjectModel& model);

// Sample object poses starting from index (inclusive) until the next ungrasp.
// All MoveTo poses and some subset of FollowTrajectory poses will be
// included.
// This is used for grasp planning: the grasp should have IK solutions for
// future poses. All returned poses are in the planning frame.
std::vector<TypedPose> GetFutureObjectPoses(
    const std::vector<task_perception_msgs::Step>& steps, const size_t index);
}  // namespace pbi

#endif  // _PBI_PROGRAM_GENERATOR_H_
