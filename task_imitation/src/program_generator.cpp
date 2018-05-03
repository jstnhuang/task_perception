// Given a demonstration as a sequence of DemoStates, generates a program to
// imitate the demonstration.
#include "task_imitation/program_generator.h"

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception/shape_detection.h"
#include "task_perception_msgs/HandState.h"
#include "task_perception_msgs/Step.h"
#include "transform_graph/graph.h"

#include "task_imitation/collision_checker.h"
#include "task_imitation/demo_state.h"
#include "task_imitation/grasp_planner.h"
#include "task_imitation/grasp_planning_context.h"
#include "task_imitation/hand_state_machine.h"
#include "task_imitation/program_constants.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using geometry_msgs::Pose;

namespace pbi {
ProgramGenerator::ProgramGenerator(
    moveit::planning_interface::MoveGroup& left_group,
    moveit::planning_interface::MoveGroup& right_group,
    LazyObjectModel::ObjectModelCache* model_cache)
    : program_(),
      start_time_(0),
      left_group_(left_group),
      right_group_(right_group),
      planning_frame_(left_group_.getPlanningFrame()),
      collision_checker_(planning_frame_),
      model_cache_(model_cache),
      is_circular_() {}

msgs::Program ProgramGenerator::Generate(
    const std::vector<task_perception_msgs::DemoState>& demo_states,
    const ObjectStateIndex& initial_runtime_objects, const Obb& table) {
  std::vector<ProgramSegment> segments = Segment(demo_states);

  ros::Time earliest;
  if (!segments.empty()) {
    earliest = segments[0].demo_states[0].stamp;
  }
  for (size_t i = 0; i < segments.size(); ++i) {
    const ros::Time& time = segments[i].demo_states[0].stamp;
    if (time < earliest) {
      earliest = time;
    }
  }
  start_time_ = earliest;

  ObjectStateIndex initial_demo_objects = GetInitialDemoObjects(demo_states);
  for (ObjectStateIndex::const_iterator it = initial_demo_objects.begin();
       it != initial_demo_objects.end(); ++it) {
    const msgs::ObjectState& obj = it->second;
    if (is_circular_.find(obj.mesh_name) == is_circular_.end()) {
      const std::string kUnusedFrame("");
      Pose kUnusedPose;
      LazyObjectModel model(obj.mesh_name, kUnusedFrame, kUnusedPose);
      model.set_object_model_cache(model_cache_);
      is_circular_[obj.mesh_name] = IsCircular(model.GetObjectModel());
      if (is_circular_[obj.mesh_name]) {
        ROS_INFO("%s is circular", obj.name.c_str());
      } else {
        ROS_INFO("%s is not circular", obj.name.c_str());
      }
    }
  }

  for (size_t i = 0; i < segments.size(); ++i) {
    ProcessSegment(segments[i], initial_runtime_objects, initial_demo_objects,
                   table);
  }

  ROS_INFO("Generated program");
  return program_;
}

std::vector<ProgramSegment> ProgramGenerator::Segment(
    const std::vector<msgs::DemoState>& demo_states) {
  std::vector<ProgramSegment> segments;
  HandStateMachine left(demo_states, msgs::Step::LEFT, collision_checker_,
                        &segments);
  HandStateMachine right(demo_states, msgs::Step::RIGHT, collision_checker_,
                         &segments);
  bool continue_left = true;
  bool continue_right = true;
  while (continue_left && continue_right) {
    continue_left = left.Step();
    continue_right = right.Step();
    ROS_ASSERT(continue_left == continue_right);
  }

  ROS_INFO("Segmented program.");
  double first_time = 0;
  for (size_t i = 0; i < segments.size(); ++i) {
    const ProgramSegment& segment = segments[i];
    if (i == 0) {
      first_time = segment.demo_states[0].stamp.toSec();
    }
    double time = segment.demo_states[0].stamp.toSec() - first_time;
    if (segment.type != msgs::Step::UNGRASP) {
      ROS_INFO("%f Segment %ld: %s %s relative to %s", time, i,
               segment.arm_name.c_str(), segment.type.c_str(),
               segment.target_object.c_str());
    } else {
      ROS_INFO("%f Segment %ld: %s %s", time, i, segment.arm_name.c_str(),
               segment.type.c_str());
    }
  }
  return segments;
}

void ProgramGenerator::ProcessSegment(
    const ProgramSegment& segment,
    const ObjectStateIndex& initial_runtime_objects,
    const ObjectStateIndex& initial_demo_objects, const Obb& table) {
  if (segment.type == msgs::Step::GRASP) {
    AddGraspStep(segment, initial_runtime_objects, table);
  } else if (segment.type == msgs::Step::UNGRASP) {
    AddUngraspStep(segment);
  } else if (segment.type == msgs::Step::MOVE_TO_POSE) {
    AddMoveToStep(segment, initial_demo_objects, initial_runtime_objects);
  } else if (segment.type == msgs::Step::FOLLOW_TRAJECTORY) {
    AddTrajectoryStep(segment, initial_runtime_objects);
  } else {
    ROS_ASSERT(false);
  }
}

void ProgramGenerator::AddGraspStep(
    const ProgramSegment& segment,
    const ObjectStateIndex& initial_runtime_objects, const Obb& table) {
  const msgs::DemoState& state = segment.demo_states[0];

  msgs::HandState hand;
  if (segment.arm_name == msgs::Step::LEFT) {
    hand = state.left_hand;
  } else if (segment.arm_name == msgs::Step::RIGHT) {
    hand = state.right_hand;
  }

  msgs::Step grasp_step;
  grasp_step.start_time = state.stamp - start_time_;
  grasp_step.arm = segment.arm_name;
  grasp_step.type = msgs::Step::GRASP;
  grasp_step.object_state = GetObjectState(state, hand.object_name);
  const std::string& object_name = grasp_step.object_state.name;

  // Plan grasp
  tg::Graph graph;
  const Pose& current_obj_pose = initial_runtime_objects.at(object_name).pose;
  graph.Add("object", tg::RefFrame("planning"), current_obj_pose);
  tg::Transform wrist_in_planning;
  graph.DescribePose(hand.wrist_pose, tg::Source("object"),
                     tg::Target("planning"), &wrist_in_planning);

  GraspPlanningContext context(wrist_in_planning.pose(), planning_frame_,
                               object_name, grasp_step.object_state.mesh_name,
                               current_obj_pose);
  context.AddObstacle(table);

  GraspPlanner grasp_planner;
  Pose grasp_in_planning = grasp_planner.Plan(context);
  graph.Add("grasp", tg::RefFrame("planning"), grasp_in_planning);
  tg::Transform grasp_in_obj;
  graph.ComputeDescription("grasp", tg::RefFrame("object"), &grasp_in_obj);

  grasp_step.ee_trajectory.push_back(grasp_in_obj.pose());
  program_.steps.push_back(grasp_step);
}

void ProgramGenerator::AddUngraspStep(const ProgramSegment& segment) {
  ROS_ASSERT(segment.demo_states.size() > 0);
  const msgs::DemoState& state = segment.demo_states[0];

  msgs::Step ungrasp_step;
  ungrasp_step.start_time = state.stamp - start_time_;
  ungrasp_step.arm = segment.arm_name;
  ungrasp_step.type = msgs::Step::UNGRASP;
  program_.steps.push_back(ungrasp_step);
}

void ProgramGenerator::AddMoveToStep(
    const ProgramSegment& segment, const ObjectStateIndex& initial_demo_objects,
    const ObjectStateIndex& initial_runtime_objects) {
  // Get grasp pose
  int prev_grasp_i = GetMostRecentGraspStep(segment.arm_name);
  ROS_ASSERT(prev_grasp_i != -1);
  const msgs::Step prev_grasp = program_.steps[prev_grasp_i];
  tg::Graph graph;
  graph.Add("gripper", tg::RefFrame("grasped object"),
            prev_grasp.ee_trajectory[0]);

  // Add transform of grasped object relative to target object
  ROS_ASSERT(segment.demo_states.size() == 2);
  const msgs::DemoState end_state(segment.demo_states[1]);
  msgs::HandState hand;
  if (segment.arm_name == msgs::Step::LEFT) {
    hand = end_state.left_hand;
  } else if (segment.arm_name == msgs::Step::RIGHT) {
    hand = end_state.right_hand;
  }

  const msgs::ObjectState grasped_obj =
      GetObjectState(end_state, hand.object_name);
  msgs::ObjectState target_obj;
  if (hand.object_name == segment.target_object) {
    // If a "move-to" is relative to itself, then move the object relative to
    // its initial pose at the time of the demonstration.
    target_obj = initial_demo_objects.at(hand.object_name);
  } else {
    target_obj = GetObjectState(end_state, segment.target_object);
  }
  graph.Add("grasped object", tg::RefFrame("camera"), grasped_obj.pose);
  graph.Add("target object", tg::RefFrame("camera"), target_obj.pose);
  tg::Transform ee_in_target;
  graph.ComputeDescription("gripper", tg::RefFrame("target object"),
                           &ee_in_target);

  Pose final_ee_in_target = ee_in_target.pose();

  // If the destination is unreachable and the object is circular, it might be
  // the case that the object has the wrong yaw value. Try rotating the object
  // about its local z-axis until we find a feasible location.
  if (is_circular_[target_obj.mesh_name]) {
    // Compute pose in planning frame
    tg::Transform grasped_obj_in_target;
    graph.ComputeDescription("grasped object", tg::RefFrame("target object"),
                             &grasped_obj_in_target);

    tg::Graph runtime_graph;
    const msgs::ObjectState& runtime_target_obj =
        initial_runtime_objects.at(target_obj.name);
    runtime_graph.Add("target object", tg::RefFrame("planning"),
                      runtime_target_obj.pose);
    runtime_graph.Add("grasped object", tg::RefFrame("target object"),
                      grasped_obj_in_target);
    runtime_graph.Add("gripper", tg::RefFrame("rotated grasped object"),
                      prev_grasp.ee_trajectory[0]);

    bool found_ik = false;
    for (int i = 0; i < 20; ++i) {
      Eigen::AngleAxisd yaw(i * M_PI / 10, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond yaw_q(yaw);
      const double kYawDegrees = yaw.angle() * 180 / M_PI;
      runtime_graph.Add("rotated grasped object",
                        tg::RefFrame("grasped object"),
                        tg::Transform(tg::Position(), yaw_q));
      tg::Transform rotated_ee_in_planning;
      runtime_graph.ComputeDescription("gripper", tg::RefFrame("planning"),
                                       &rotated_ee_in_planning);
      if (HasIk(segment.arm_name, rotated_ee_in_planning.pose())) {
        ROS_INFO("IK found with yaw of %f", kYawDegrees);
        tg::Transform rotated_ee_in_target;
        runtime_graph.ComputeDescription(
            "gripper", tg::RefFrame("target object"), &rotated_ee_in_target);
        final_ee_in_target = rotated_ee_in_target.pose();
        found_ik = true;
        break;
      } else {
        ROS_WARN("No IK for yaw of %f", kYawDegrees);
      }
    }
    if (!found_ik) {
      ROS_ERROR("No IK solution found for MoveTo step!");
    }
  }

  const ros::Time step_start(segment.demo_states[0].stamp);
  msgs::Step move_step;
  move_step.start_time = step_start - start_time_ + ros::Duration(0.03);
  move_step.arm = segment.arm_name;
  move_step.type = msgs::Step::MOVE_TO_POSE;
  move_step.object_state = target_obj;
  move_step.ee_trajectory.push_back(final_ee_in_target);
  move_step.times_from_start.push_back(end_state.stamp - step_start);
  program_.steps.push_back(move_step);
}

void ProgramGenerator::AddTrajectoryStep(
    const ProgramSegment& segment,
    const ObjectStateIndex& initial_runtime_objects) {
  ROS_ASSERT(segment.demo_states.size() > 0);
  const msgs::DemoState& start_state = segment.demo_states[0];

  // Set up all fields of trajectory step other than the trajectory itself.
  msgs::Step traj_step;
  traj_step.start_time = start_state.stamp - start_time_ + ros::Duration(0.03);
  traj_step.arm = segment.arm_name;
  traj_step.type = msgs::Step::FOLLOW_TRAJECTORY;
  traj_step.object_state = initial_runtime_objects.at(segment.target_object);

  // Get grasp pose
  int prev_grasp_i = GetMostRecentGraspStep(segment.arm_name);
  ROS_ASSERT(prev_grasp_i != -1);
  const msgs::Step prev_grasp = program_.steps[prev_grasp_i];

  tg::Graph graph;
  graph.Add("gripper", tg::RefFrame("rotated grasped object"),
            prev_grasp.ee_trajectory[0]);
  graph.Add("rotated grasped object", tg::RefFrame("grasped object"),
            tg::Transform());
  const msgs::ObjectState& target_obj_at_runtime =
      initial_runtime_objects.at(segment.target_object);
  graph.Add("target object", tg::RefFrame("planning"),
            target_obj_at_runtime.pose);

  // Compute trajectory relative to the target object
  ROS_ASSERT(!segment.demo_states.empty());
  msgs::HandState hand;
  if (segment.arm_name == msgs::Step::LEFT) {
    hand = segment.demo_states[0].left_hand;
  } else if (segment.arm_name == msgs::Step::RIGHT) {
    hand = segment.demo_states[0].right_hand;
  }
  const std::string& hand_obj_name(hand.object_name);

  for (size_t i = 0; i < segment.demo_states.size(); ++i) {
    const msgs::DemoState state = segment.demo_states[i];
    const msgs::ObjectState grasped_obj = GetObjectState(state, hand_obj_name);
    const msgs::ObjectState target_obj =
        GetObjectState(state, segment.target_object);
    graph.Add("grasped object", tg::RefFrame("camera"), grasped_obj.pose);
    graph.Add("target object", tg::RefFrame("camera"), target_obj.pose);
    tg::Transform ee_in_target;
    graph.ComputeDescription(tg::LocalFrame("gripper"),
                             tg::RefFrame("target object"), &ee_in_target);
    if (is_circular_[grasped_obj.mesh_name]) {
      // Check to see if IK failed. If so, try to correct it.
      tg::Transform ee_in_planning;
      graph.ComputeDescription("gripper", tg::RefFrame("planning"),
                               &ee_in_planning);
      if (!HasIk(segment.arm_name, ee_in_planning.pose())) {
        ROS_WARN("No IK, searching...");
        // If initial pose has no IK, then search for IK in both directions in
        // order: +10, -10, +20, -20, ..., +170, -170.
        bool found_ik = false;
        for (int yaw_i = 0; yaw_i < 17; ++yaw_i) {
          Eigen::AngleAxisd yaw(yaw_i * 10 * M_PI / 180,
                                Eigen::Vector3d::UnitZ());
          Eigen::Quaterniond yaw_q(yaw);
          const double kYawDegrees = yaw.angle() * 180 / M_PI;
          graph.Add("rotated grasped object", tg::RefFrame("grasped object"),
                    tg::Transform(tg::Position(), yaw_q));
          graph.ComputeDescription("gripper", tg::RefFrame("planning"),
                                   &ee_in_planning);
          if (HasIk(segment.arm_name, ee_in_planning.pose())) {
            ROS_INFO("IK found with yaw of %f", kYawDegrees);
            found_ik = true;
            break;
          }

          Eigen::AngleAxisd neg_yaw(yaw_i * M_PI / 10,
                                    Eigen::Vector3d::UnitZ());
          Eigen::Quaterniond neg_yaw_q(yaw);
          const double kNegYawDegrees = neg_yaw.angle() * 180 / M_PI;
          graph.Add("rotated grasped object", tg::RefFrame("grasped object"),
                    tg::Transform(tg::Position(), neg_yaw_q));
          graph.ComputeDescription("gripper", tg::RefFrame("planning"),
                                   &ee_in_planning);
          if (HasIk(segment.arm_name, ee_in_planning.pose())) {
            ROS_INFO("IK found with yaw of %f", kNegYawDegrees);
            found_ik = true;
            break;
          }
        }
        if (!found_ik) {
          ROS_ERROR("Failed to find IK for trajectory point.");
        }
      }
    }
    traj_step.ee_trajectory.push_back(ee_in_target.pose());
    if (i == 0) {
      traj_step.times_from_start.push_back(ros::Duration(0));
    } else {
      ros::Time prev_time(segment.demo_states[i - 1].stamp);
      ros::Duration dt = state.stamp - prev_time;
      ros::Duration current_time = traj_step.times_from_start[i - 1] + dt;
      traj_step.times_from_start.push_back(current_time);
    }
  }

  program_.steps.push_back(traj_step);
}

int ProgramGenerator::GetMostRecentGraspStep(const std::string& arm_name) {
  for (int i = program_.steps.size() - 1; i >= 0; --i) {
    const msgs::Step& step = program_.steps[i];
    if (step.arm == arm_name && step.type == msgs::Step::GRASP) {
      return i;
    }
  }
  return -1;
}

bool ProgramGenerator::HasIk(const std::string& arm_name,
                             const geometry_msgs::Pose& ee_pose) {
  moveit::planning_interface::MoveGroup& group =
      arm_name == msgs::Step::LEFT ? left_group_ : right_group_;
  const std::string group_name =
      arm_name == msgs::Step::LEFT ? "left_arm" : "right_arm";
  robot_state::RobotState kinematic_state(group.getRobotModel());
  return kinematic_state.setFromIK(
      group.getRobotModel()->getJointModelGroup(group_name), ee_pose, 10, 0.1);
}

ProgramGenerator::ObjectStateIndex GetInitialDemoObjects(
    const std::vector<msgs::DemoState>& demo_states) {
  ProgramGenerator::ObjectStateIndex index;
  for (size_t i = 0; i < demo_states.size(); ++i) {
    const msgs::DemoState& demo_state = demo_states[i];
    for (size_t j = 0; j < demo_state.object_states.size(); ++j) {
      const msgs::ObjectState& obj = demo_state.object_states[j];
      if (index.find(obj.name) == index.end()) {
        index[obj.name] = obj;
      }
    }
  }
  return index;
}
}  // namespace pbi
