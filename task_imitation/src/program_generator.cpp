// Given a demonstration as a sequence of DemoStates, generates a program to
// imitate the demonstration.
#include "task_imitation/program_generator.h"

#include <algorithm>

#include "boost/foreach.hpp"
#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "rapid_ros/params.h"
#include "rapid_utils/vector3.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception/pr2_gripper_model.h"
#include "task_perception_msgs/HandState.h"
#include "transform_graph/graph.h"

#include "task_imitation/demo_state.h"
#include "task_imitation/grasp_planner.h"
#include "task_imitation/grasp_planning_context.h"
#include "task_imitation/hand_state_machine.h"
#include "task_imitation/ik.h"
#include "task_imitation/program_constants.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using geometry_msgs::Pose;

namespace pbi {
ProgramGenerator::ProgramGenerator(
    moveit::planning_interface::MoveGroup& left_group,
    moveit::planning_interface::MoveGroup& right_group,
    ObjectModelCache* model_cache, const Pr2GripperViz& gripper_viz)
    : program_(),
      start_time_(0),
      left_group_(left_group),
      right_group_(right_group),
      planning_frame_(left_group_.getPlanningFrame()),
      model_cache_(model_cache),
      collision_checker_(planning_frame_, model_cache_),
      gripper_viz_(gripper_viz),
      nh_(),
      gripper_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "program_generator_marker_array", 100)) {}

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

  // Three-step process:
  // 1) Pre-compute the object trajectories and store them in the program
  // 2) Plan grasps for grasp steps (this can look at the pre-computed object
  //    trajectories and find a grasp that will work for future poses).
  // 3) Update MoveTo and FollowTrajectory steps with the planned grasp
  for (size_t i = 0; i < segments.size(); ++i) {
    ProcessSegmentWithoutGrasps(segments, i, initial_runtime_objects,
                                initial_demo_objects);
  }

  // Segment index is not necessarily the same as step index, since single-point
  // trajectories are skipped and do not turn into steps.

  for (size_t i = 0, step_i = 0; i < segments.size(); i++) {
    const ProgramSegment& segment = segments[i];
    // Skip trajectories with 1 or fewer points
    if (segment.type == msgs::Step::FOLLOW_TRAJECTORY &&
        segment.demo_states.size() <= 1) {
      continue;
    }
    if (segment.type == msgs::Step::GRASP) {
      const msgs::DemoState& state = segment.demo_states[0];
      const msgs::HandState& hand(segment.arm_name == msgs::Step::LEFT
                                      ? state.left_hand
                                      : state.right_hand);
      const msgs::ObjectState& hand_obj =
          initial_runtime_objects.at(hand.object_name);
      std::vector<Obb> obstacles;
      obstacles.push_back(table);
      for (ObjectStateIndex::const_iterator it =
               initial_runtime_objects.begin();
           it != initial_runtime_objects.end(); ++it) {
        if (it->first == hand.object_name) {
          continue;
        }
        if ((rapid::AsVector3d(hand_obj.pose.position) -
             rapid::AsVector3d(it->second.pose.position))
                .norm() > 0.1) {
          ROS_INFO("%s is too far away to be an obstacle of %s",
                   it->first.c_str(), hand.object_name.c_str());
          continue;
        }

        LazyObjectModel obj_model(it->second.mesh_name, planning_frame_,
                                  it->second.pose);
        obj_model.set_object_model_cache(model_cache_);
        Obb obb;
        obb.pose = obj_model.center_pose();
        obb.dims = obj_model.scale();
        obstacles.push_back(obb);
      }
      program_.steps[step_i] =
          PlanGrasp(program_.steps, step_i, hand.wrist_pose, obstacles);
    }
    step_i++;
  }

  Pose left_gripper_in_obj;
  Pose right_gripper_in_obj;
  Eigen::Quaterniond left_obj_rotation = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond right_obj_rotation = Eigen::Quaterniond::Identity();
  std::string prev_left_step("");
  std::string prev_right_step("");
  for (size_t i = 0, step_i = 0; i < segments.size(); ++i) {
    const ProgramSegment& segment = segments[i];
    if (segment.type == msgs::Step::FOLLOW_TRAJECTORY &&
        segment.demo_states.size() <= 1) {
      continue;
    }

    const msgs::Step& step = program_.steps[step_i];
    ROS_ASSERT(segment.type == step.type);

    Eigen::Quaterniond obj_rotation_in = Eigen::Quaterniond::Identity();
    if (segment.arm_name == msgs::Step::LEFT) {
      obj_rotation_in = left_obj_rotation;
    } else {
      obj_rotation_in = right_obj_rotation;
    }

    if (segment.type == msgs::Step::GRASP) {
      if (segment.arm_name == msgs::Step::LEFT) {
        left_gripper_in_obj = step.ee_trajectory[0];
      } else if (segment.arm_name == msgs::Step::RIGHT) {
        right_gripper_in_obj = step.ee_trajectory[0];
      }
    } else if (segment.type == msgs::Step::MOVE_TO_POSE) {
      const Pose& gripper_in_obj = segment.arm_name == msgs::Step::LEFT
                                       ? left_gripper_in_obj
                                       : right_gripper_in_obj;
      Eigen::Quaterniond obj_rotation_out;
      program_.steps[step_i] = ParameterizeMoveToWithGrasp(
          segment, step, gripper_in_obj, obj_rotation_in, &obj_rotation_out);
      if (segment.arm_name == msgs::Step::LEFT) {
        left_obj_rotation = obj_rotation_out;
      } else {
        right_obj_rotation = obj_rotation_out;
      }
    } else if (segment.type == msgs::Step::FOLLOW_TRAJECTORY) {
      const Pose& gripper_in_obj = segment.arm_name == msgs::Step::LEFT
                                       ? left_gripper_in_obj
                                       : right_gripper_in_obj;
      Eigen::Quaterniond obj_rotation_out;
      const std::string prev_step_type(segment.arm_name == msgs::Step::LEFT
                                           ? prev_left_step
                                           : prev_right_step);
      program_.steps[step_i] = ParameterizeTrajectoryWithGrasp(
          segment, step, gripper_in_obj, table, obj_rotation_in, prev_step_type,
          &obj_rotation_out);
      if (segment.arm_name == msgs::Step::LEFT) {
        left_obj_rotation = obj_rotation_out;
      } else {
        right_obj_rotation = obj_rotation_out;
      }
    } else if (segment.type == msgs::Step::UNGRASP) {
      if (segment.arm_name == msgs::Step::LEFT) {
        left_obj_rotation = Eigen::Quaterniond::Identity();
      } else {
        right_obj_rotation = Eigen::Quaterniond::Identity();
      }
    }
    step_i++;
    if (segment.arm_name == msgs::Step::LEFT) {
      prev_left_step = segment.type;
    } else if (segment.arm_name == msgs::Step::RIGHT) {
      prev_right_step = segment.type;
    }
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
    ROS_INFO("%f Segment %ld: %s %s relative to %s", time, i,
             segment.arm_name.c_str(), segment.type.c_str(),
             segment.target_object.c_str());
  }
  return segments;
}

void ProgramGenerator::ProcessSegmentWithoutGrasps(
    const std::vector<ProgramSegment>& segments, const size_t index,
    const ObjectStateIndex& initial_runtime_objects,
    const ObjectStateIndex& initial_demo_objects) {
  const ProgramSegment& segment = segments[index];
  if (segment.type == msgs::Step::GRASP) {
    AddGraspStep(segment, initial_runtime_objects);
  } else if (segment.type == msgs::Step::UNGRASP) {
    AddUngraspStep(segment);
  } else if (segment.type == msgs::Step::MOVE_TO_POSE) {
    AddMoveToStep(segment, initial_demo_objects, initial_runtime_objects);
  } else if (segment.type == msgs::Step::FOLLOW_TRAJECTORY) {
    if (segment.demo_states.size() > 1) {
      AddTrajectoryStep(segment, initial_demo_objects, initial_runtime_objects);
    } else {
      ROS_WARN("Ignored trajectory (0-indexed segment %zu) with single point.",
               index);
    }
  } else {
    ROS_ASSERT(false);
  }
}

msgs::Step ProgramGenerator::PlanGrasp(const std::vector<msgs::Step>& steps,
                                       const size_t index,
                                       const Pose& wrist_in_obj,
                                       const std::vector<Obb>& obstacles) {
  const msgs::Step& grasp_step = steps[index];
  tg::Graph graph;
  graph.Add("object", tg::RefFrame("planning"), grasp_step.object_state.pose);
  tg::Transform wrist_in_planning;
  graph.DescribePose(wrist_in_obj, tg::Source("object"), tg::Target("planning"),
                     &wrist_in_planning);

  moveit::planning_interface::MoveGroup& group =
      grasp_step.arm == msgs::Step::LEFT ? left_group_ : right_group_;
  const std::vector<TypedPose> future_poses =
      GetFutureObjectPoses(steps, index);

  ROS_INFO("Planning with %zu future poses", future_poses.size());
  GraspPlanningContext context(wrist_in_planning.pose(), planning_frame_,
                               grasp_step.object_state.mesh_name,
                               grasp_step.object_state.pose, future_poses,
                               &group, model_cache_);
  BOOST_FOREACH (const Obb& obstacle, obstacles) {
    context.AddObstacle(obstacle);
  }

  GraspPlanner grasp_planner(gripper_viz_);
  Pose grasp_in_planning = grasp_planner.Plan(context);
  graph.Add("grasp", tg::RefFrame("planning"), grasp_in_planning);
  tg::Transform grasp_in_obj;
  graph.ComputeDescription("grasp", tg::RefFrame("object"), &grasp_in_obj);

  msgs::Step updated_step = grasp_step;
  updated_step.ee_trajectory.push_back(grasp_in_obj.pose());
  return updated_step;
}

void ProgramGenerator::AddGraspStep(
    const ProgramSegment& segment,
    const ObjectStateIndex& initial_runtime_objects) {
  const msgs::DemoState& state = segment.demo_states[0];
  const msgs::HandState& hand(segment.arm_name == msgs::Step::LEFT
                                  ? state.left_hand
                                  : state.right_hand);

  // const msgs::ObjectState target_object =
  //    GetObjectState(state, hand.object_name);
  const msgs::ObjectState target_object =
      initial_runtime_objects.at(hand.object_name);

  msgs::Step grasp_step;
  grasp_step.start_time = state.stamp - start_time_;
  grasp_step.arm = segment.arm_name;
  grasp_step.type = msgs::Step::GRASP;
  grasp_step.object_state = target_object;

  program_.steps.push_back(grasp_step);
}

void ProgramGenerator::AddUngraspStep(const ProgramSegment& segment) {
  const msgs::DemoState& state = segment.demo_states[0];

  msgs::Step ungrasp_step;
  ungrasp_step.start_time = state.stamp - start_time_;
  ungrasp_step.arm = segment.arm_name;
  ungrasp_step.type = msgs::Step::UNGRASP;
  ungrasp_step.object_state.name = segment.target_object;
  program_.steps.push_back(ungrasp_step);
}

void ProgramGenerator::AddMoveToStep(
    const ProgramSegment& segment, const ObjectStateIndex& initial_demo_objects,
    const ObjectStateIndex& initial_runtime_objects) {
  // Get transform of grasped object relative to target object
  ROS_ASSERT(segment.demo_states.size() == 2);
  const msgs::DemoState& end_state(segment.demo_states[1]);
  const msgs::HandState& hand(segment.arm_name == msgs::Step::LEFT
                                  ? end_state.left_hand
                                  : end_state.right_hand);

  tg::Graph graph;
  const msgs::ObjectState demo_grasped_obj =
      GetObjectState(end_state, hand.object_name);
  graph.Add("grasped object", tg::RefFrame("camera"), demo_grasped_obj.pose);
  const msgs::ObjectState& demo_target_obj =
      initial_demo_objects.at(segment.target_object);
  graph.Add("target object", tg::RefFrame("camera"), demo_target_obj.pose);

  tg::Transform grasped_obj_in_target;
  graph.ComputeDescription("grasped object", tg::RefFrame("target object"),
                           &grasped_obj_in_target);

  const msgs::DemoState& start_state(segment.demo_states[0]);
  const ros::Time step_start(start_state.stamp);
  msgs::Step move_step;
  move_step.start_time = step_start - start_time_ + ros::Duration(0.03);
  move_step.arm = segment.arm_name;
  move_step.type = msgs::Step::MOVE_TO_POSE;
  move_step.object_state = initial_runtime_objects.at(segment.target_object);
  move_step.ee_trajectory.push_back(grasped_obj_in_target.pose());
  move_step.times_from_start.push_back(end_state.stamp - step_start);
  program_.steps.push_back(move_step);
}

msgs::Step ProgramGenerator::ParameterizeMoveToWithGrasp(
    const ProgramSegment& segment, const msgs::Step& move_step,
    const Pose& gripper_in_obj, const Eigen::Quaterniond& obj_rotation_in,
    Eigen::Quaterniond* obj_rotation_out) {
  *obj_rotation_out = obj_rotation_in;
  // Set up transform graph
  tg::Graph graph;
  graph.Add("target object", tg::RefFrame("planning"),
            move_step.object_state.pose);
  graph.Add("grasped object", tg::RefFrame("target object"),
            move_step.ee_trajectory[0]);
  graph.Add("rotated grasped object", tg::RefFrame("grasped object"),
            tg::Transform(tg::Position(), obj_rotation_in));
  graph.Add("gripper", tg::RefFrame("rotated grasped object"), gripper_in_obj);

  // Compute gripper pose relative to target.
  tg::Transform ee_in_target;
  graph.ComputeDescription("gripper", tg::RefFrame("target object"),
                           &ee_in_target);
  Pose final_ee_in_target = ee_in_target.pose();

  // If the destination is unreachable and the object is circular, it might be
  // the case that the object has the wrong yaw value. Try rotating the object
  // about its local z-axis until we find a feasible location.
  // The best location is the one either most to the left for the left arm or
  // most to the right for the right arm.
  ROS_ASSERT(move_step.ee_trajectory.size() > 0);
  const std::string unused_frame("");
  const Pose unused_pose;
  const msgs::DemoState& state = segment.demo_states[0];
  const msgs::HandState& hand =
      segment.arm_name == msgs::Step::LEFT ? state.left_hand : state.right_hand;
  const msgs::ObjectState grasped_obj = GetObjectState(state, hand.object_name);
  LazyObjectModel grasped_model(grasped_obj.mesh_name, unused_frame,
                                unused_pose);
  grasped_model.set_object_model_cache(model_cache_);
  bool is_grasped_obj_circular = grasped_model.IsCircular();
  if (is_grasped_obj_circular) {
    moveit::planning_interface::MoveGroup& move_group(
        move_step.arm == msgs::Step::LEFT ? left_group_ : right_group_);
    bool found_ik = false;
    double best_dist = std::numeric_limits<double>::max();
    Eigen::Vector2d ideal_xy;
    ideal_xy << 0, 1;
    if (segment.arm_name == msgs::Step::LEFT) {
      ideal_xy.y() *= -1;
    }
    ideal_xy.normalize();
    for (int i = 0; i < 11; i++) {
      for (int sign = 1; sign >= -1; sign -= 2) {
        Eigen::AngleAxisd yaw(sign * i * M_PI / 10, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond yaw_q(yaw);
        graph.Add("rotated grasped object", tg::RefFrame("grasped object"),
                  tg::Transform(tg::Position(), yaw_q));
        tg::Transform rotated_ee_in_planning;
        graph.ComputeDescription("gripper", tg::RefFrame("planning"),
                                 &rotated_ee_in_planning);
        if (HasIk(move_group, rotated_ee_in_planning.pose())) {
          Eigen::Matrix3d orientation =
              rotated_ee_in_planning.orientation().matrix();
          Eigen::Vector2d xy = orientation.topLeftCorner(2, 1);
          xy.normalize();
          double sq_dist = (ideal_xy - xy).squaredNorm();
          if (sq_dist < best_dist) {
            best_dist = sq_dist;
            tg::Transform rotated_ee_in_target;
            graph.ComputeDescription("gripper", tg::RefFrame("target object"),
                                     &rotated_ee_in_target);
            final_ee_in_target = rotated_ee_in_target.pose();
            *obj_rotation_out = yaw_q;
            found_ik = true;
          }
        }
        if (i == 0 || i == 10) {
          break;
        }
      }
    }
    if (!found_ik) {
      ROS_ERROR("No IK solution found for MoveTo step!");
    }
  }

  msgs::Step updated_step = move_step;
  updated_step.ee_trajectory[0] = final_ee_in_target;
  return updated_step;
}

void ProgramGenerator::AddTrajectoryStep(
    const ProgramSegment& segment, const ObjectStateIndex& initial_demo_objects,
    const ObjectStateIndex& initial_runtime_objects) {
  ROS_ASSERT(segment.demo_states.size() > 0);
  const msgs::DemoState& start_state = segment.demo_states[0];

  // Set up all fields of trajectory step other than the trajectory itself.
  msgs::Step traj_step;
  traj_step.start_time = start_state.stamp - start_time_ + ros::Duration(0.03);
  traj_step.arm = segment.arm_name;
  traj_step.type = msgs::Step::FOLLOW_TRAJECTORY;
  traj_step.object_state = initial_runtime_objects.at(segment.target_object);

  // Compute object trajectory
  // Get target object pose at demo time and at runtime.
  tg::Graph graph;
  const msgs::ObjectState& demo_target_obj =
      initial_demo_objects.at(segment.target_object);
  graph.Add("target object", tg::RefFrame("camera"), demo_target_obj.pose);

  ROS_ASSERT(!segment.demo_states.empty());
  const msgs::HandState& hand = segment.arm_name == msgs::Step::LEFT
                                    ? start_state.left_hand
                                    : start_state.right_hand;
  const std::string& hand_obj_name(hand.object_name);

  for (size_t i = 0; i < segment.demo_states.size(); ++i) {
    const msgs::DemoState& state = segment.demo_states[i];
    const msgs::ObjectState demo_grasped_obj =
        GetObjectState(state, hand_obj_name);
    graph.Add("grasped object", tg::RefFrame("camera"), demo_grasped_obj.pose);
    tg::Transform obj_in_target;
    graph.ComputeDescription("grasped object", tg::RefFrame("target object"),
                             &obj_in_target);
    traj_step.ee_trajectory.push_back(obj_in_target.pose());
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

msgs::Step ProgramGenerator::ParameterizeTrajectoryWithGrasp(
    const ProgramSegment& segment, const msgs::Step& traj_step,
    const Pose& gripper_in_obj, const Obb& table,
    const Eigen::Quaterniond& obj_rotation_in,
    const std::string& prev_step_type, Eigen::Quaterniond* obj_rotation_out) {
  *obj_rotation_out = obj_rotation_in;
  tg::Graph graph;
  graph.Add("gripper", tg::RefFrame("rotated grasped object"), gripper_in_obj);
  graph.Add("rotated grasped object", tg::RefFrame("prerotated grasped object"),
            tg::Transform(tg::Position(), obj_rotation_in));
  graph.Add("prerotated grasped object", tg::RefFrame("grasped object"),
            tg::Transform::Identity());
  graph.Add("target object", tg::RefFrame("planning"),
            traj_step.object_state.pose);
  graph.Add("postgrasp", tg::RefFrame("gripper"),
            tg::Transform(tg::Position(-kPostgraspDistance, 0, 0),
                          tg::Orientation()));

  ROS_ASSERT(traj_step.ee_trajectory.size() > 0);
  const std::string unused_frame("");
  const Pose unused_pose;
  const msgs::DemoState& state = segment.demo_states[0];
  const msgs::HandState& hand =
      segment.arm_name == msgs::Step::LEFT ? state.left_hand : state.right_hand;
  const msgs::ObjectState grasped_obj = GetObjectState(state, hand.object_name);
  LazyObjectModel grasped_model(grasped_obj.mesh_name, unused_frame,
                                unused_pose);
  grasped_model.set_object_model_cache(model_cache_);
  bool is_grasped_obj_circular = grasped_model.IsCircular();

  Pr2GripperModel gripper_model;
  moveit::planning_interface::MoveGroup& move_group(
      segment.arm_name == msgs::Step::LEFT ? left_group_ : right_group_);
  msgs::Step updated_step = traj_step;
  for (size_t i = 0; i < traj_step.ee_trajectory.size(); ++i) {
    graph.Add("grasped object", tg::RefFrame("target object"),
              traj_step.ee_trajectory[i]);
    tg::Transform gripper_in_target;
    graph.ComputeDescription("gripper", tg::RefFrame("target object"),
                             &gripper_in_target);
    Pose final_gripper_in_target = gripper_in_target.pose();

    if (is_grasped_obj_circular) {
      bool found_ik = false;
      int best_yaw_i = 0;
      bool is_left = segment.arm_name == msgs::Step::LEFT;
      double best_y = is_left ? -std::numeric_limits<double>::max()
                              : std::numeric_limits<double>::max();
      bool is_last_traj_point = i + 1 == traj_step.ee_trajectory.size();

      // Disallow yaw rotations until the object has moved a bit
      Pose current_pose = traj_step.ee_trajectory[i];
      Pose start_pose = traj_step.ee_trajectory[0];
      Eigen::Vector3d current_pos = rapid::AsVector3d(current_pose.position);
      Eigen::Vector3d start_pos = rapid::AsVector3d(start_pose.position);
      double movement_threshold = rapid::GetDoubleParamOrThrow(
          "task_imitation/traj_movement_threshold");
      bool has_moved = (current_pos - start_pos).norm() > movement_threshold;

      for (int sign = 1; sign >= -1; sign -= 2) {
        int max_yaw = 10;
        // Force yaw to stay the same if, after a grasp, the object has not
        // moved much. This unnecessary movements at the beginning of a
        // trajectory.
        if (prev_step_type == msgs::Step::GRASP && !has_moved) {
          max_yaw = 0;
        }

        int yaw_start = sign == 1 ? 0 : 1;
        int yaw_end = sign == 1 ? max_yaw : max_yaw;
        for (int yaw_i = yaw_start; yaw_i <= yaw_end; yaw_i++) {
          Eigen::AngleAxisd yaw(sign * yaw_i * M_PI / 10,
                                Eigen::Vector3d::UnitZ());
          Eigen::Quaterniond yaw_q(yaw);
          graph.Add("rotated grasped object",
                    tg::RefFrame("prerotated grasped object"),
                    tg::Transform(tg::Position(), yaw_q));
          tg::Transform ee_in_planning;
          graph.ComputeDescription("gripper", tg::RefFrame("planning"),
                                   &ee_in_planning);

          gripper_model.set_pose(ee_in_planning.pose());
          double pos_y = ee_in_planning.position().y();
          // If this is the last trajectory point, then also consider if a post
          // grasp pose is reachable.
          bool is_ready_for_ungrasp = true;
          if (is_last_traj_point) {
            tg::Transform postgrasp;
            graph.ComputeDescription("postgrasp", tg::RefFrame("planning"),
                                     &postgrasp);
            is_ready_for_ungrasp = HasIk(move_group, postgrasp.pose());
          }
          bool has_ik = HasIk(move_group, ee_in_planning.pose());
          bool is_colliding =
              gripper_model.IsBodyCollidingWithObb(table.pose, table.dims);
          if (has_ik && !is_colliding && is_ready_for_ungrasp) {
            found_ik = true;

            // ROS_INFO("%zu yaw_i: %d, pos_y: %f", i, sign * yaw_i, pos_y);
            if ((is_left && pos_y > best_y) || (!is_left && pos_y < best_y)) {
              best_yaw_i = sign * yaw_i;
              best_y = pos_y;
              tg::Transform gripper_in_rotated_target;
              graph.ComputeDescription("gripper", tg::RefFrame("target object"),
                                       &gripper_in_rotated_target);
              final_gripper_in_target = gripper_in_rotated_target.pose();
              if (is_last_traj_point) {
                tg::Transform obj_rotation;
                graph.ComputeDescription("rotated grasped object",
                                         tg::RefFrame("grasped object"),
                                         &obj_rotation);
                *obj_rotation_out = obj_rotation.orientation().quaternion();
              }
            } else {
              // If we are no longer improving the gripper's direction, stop
              // searching in this direction.
              break;
            }
          } else if (found_ik) {
            // ROS_INFO(
            //    "%zu yaw_i: %d, pos_y: %f (but has ik %d, not colliding: %d, "
            //    "ungrasp: %d) (done with this direction)",
            //    i, sign * yaw_i, pos_y, has_ik, !is_colliding,
            //    is_ready_for_ungrasp);

            // If IK was previously found, but not on this yaw angle, then skip
            break;
          } else {
            // ROS_INFO(
            //    "%zu yaw_i: %d, pos_y: %f (but has ik %d, not colliding: %d, "
            //    "ungrasp: %d)",
            //    i, sign * yaw_i, pos_y, has_ik, !is_colliding,
            //    is_ready_for_ungrasp);
          }
        }
      }

      if (!found_ik) {
        ROS_ERROR("Failed to find IK for trajectory point %zu", i + 1);

      } else {
        // Adopt the best yaw from earlier as the pre-rotation.
        // ROS_INFO("%zu Adopting yaw_i: %d, best_y: %f", i, best_yaw_i,
        // best_y);
        Eigen::AngleAxisd yaw(best_yaw_i * M_PI / 10, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond yaw_q(yaw);
        tg::Transform rotation_in_grasped_obj;
        graph.DescribePose(tg::Transform(tg::Position(), yaw_q),
                           tg::Source("prerotated grasped object"),
                           tg::Target("grasped object"),
                           &rotation_in_grasped_obj);
        graph.Add("prerotated grasped object", tg::RefFrame("grasped object"),
                  rotation_in_grasped_obj);
      }
    }
    updated_step.ee_trajectory[i] = final_gripper_in_target;
  }

  return updated_step;
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

std::vector<TypedPose> GetFutureObjectPoses(
    const std::vector<msgs::Step>& steps, const size_t index) {
  int sample_every =
      rapid::GetIntParamOrThrow("task_imitation/sample_every_nth_future_pose");
  const msgs::Step& start_step = steps[index];

  std::vector<TypedPose> future_poses;
  std::map<std::string, Pose> poses;
  for (size_t i = index; i < steps.size(); i++) {
    const msgs::Step& step = steps[i];
    if (step.arm != start_step.arm) {
      continue;
    }
    if (step.type == msgs::Step::GRASP) {
      tg::Graph graph;
      graph.Add("pose", tg::RefFrame("planning"), step.object_state.pose);
      tg::Transform pregrasp;
      graph.DescribePose(tg::Transform(tg::Position(-kPregraspDistance, 0, 0),
                                       tg::Orientation()),
                         tg::Source("pose"), tg::Target("planning"), &pregrasp);
      TypedPose typed_pose;
      typed_pose.type = TypedPose::PREGRASP;
      typed_pose.pose = pregrasp.pose();
      future_poses.push_back(typed_pose);
      typed_pose.type = TypedPose::GRASP;
      typed_pose.pose = step.object_state.pose;
      future_poses.push_back(typed_pose);

      poses[step.object_state.name] = step.object_state.pose;
    } else if (step.type == msgs::Step::UNGRASP) {
      if (poses.find(step.object_state.name) != poses.end()) {
        TypedPose typed_pose;
        typed_pose.type = TypedPose::UNGRASP;
        typed_pose.pose = poses.at(step.object_state.name);
        future_poses.push_back(typed_pose);

        tg::Graph graph;
        graph.Add("ungrasp", tg::RefFrame("planning"), typed_pose.pose);
        graph.Add("postgrasp", tg::RefFrame("ungrasp"),
                  tg::Transform(tg::Position(-kPostgraspDistance, 0, 0),
                                tg::Orientation()));
        tg::Transform postgrasp_in_planning;
        graph.ComputeDescription("postgrasp", tg::RefFrame("planning"),
                                 &postgrasp_in_planning);
        typed_pose.type = TypedPose::POSTGRASP;
        typed_pose.pose = postgrasp_in_planning.pose();
        future_poses.push_back(typed_pose);
      } else {
        ROS_ERROR("Unable to get last pose of %s",
                  step.object_state.name.c_str());
      }
      break;
    } else if (step.type == msgs::Step::MOVE_TO_POSE) {
      tg::Graph graph;
      graph.Add("target", tg::RefFrame("planning"), step.object_state.pose);
      const Pose& obj_in_target = step.ee_trajectory[0];
      graph.Add("grasped object", tg::RefFrame("target"), obj_in_target);
      tg::Transform obj_in_planning;
      graph.ComputeDescription("grasped object", tg::RefFrame("planning"),
                               &obj_in_planning);
      TypedPose typed_pose;
      typed_pose.type = TypedPose::MOVE_TO;
      typed_pose.pose = obj_in_planning.pose();
      future_poses.push_back(typed_pose);

      poses[step.object_state.name] = step.object_state.pose;
    } else if (step.type == msgs::Step::FOLLOW_TRAJECTORY) {
      tg::Graph graph;
      graph.Add("target", tg::RefFrame("planning"), step.object_state.pose);
      for (size_t i = 1; i < step.ee_trajectory.size(); i += sample_every) {
        const Pose& obj_in_target = step.ee_trajectory[i];
        graph.Add("grasped object", tg::RefFrame("target"), obj_in_target);
        tg::Transform obj_in_planning;
        graph.ComputeDescription("grasped object", tg::RefFrame("planning"),
                                 &obj_in_planning);
        TypedPose typed_pose;
        typed_pose.type = TypedPose::TRAJECTORY;
        typed_pose.pose = obj_in_planning.pose();
        future_poses.push_back(typed_pose);
      }

      // Record the last position of the object.
      if (step.ee_trajectory.size() > 0) {
        graph.Add("grasped object", tg::RefFrame("target"),
                  step.ee_trajectory.back());
        tg::Transform obj_in_planning;
        graph.ComputeDescription("grasped object", tg::RefFrame("planning"),
                                 &obj_in_planning);
        poses[step.object_state.name] = obj_in_planning.pose();
      }
    }
  }
  return future_poses;
}
}  // namespace pbi
