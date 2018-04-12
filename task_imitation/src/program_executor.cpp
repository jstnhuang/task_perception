#include "task_imitation/program_executor.h"

#include "boost/optional.hpp"
#include "moveit/robot_state/conversions.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "rapid_ros/params.h"
#include "std_msgs/Bool.h"
#include "task_perception_msgs/ProgramSlice.h"
#include "task_perception_msgs/ProgramSlices.h"
#include "task_perception_msgs/Step.h"
#include "task_utils/pr2_gripper_viz.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "transform_graph/graph.h"

#include "task_imitation/bimanual_manipulation.h"
#include "task_imitation/program_constants.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using boost::optional;
using geometry_msgs::Pose;
using trajectory_msgs::JointTrajectory;
using trajectory_msgs::JointTrajectoryPoint;
using task_perception_msgs::ProgramSlice;

namespace pbi {
ProgramExecutor::ProgramExecutor(
    moveit::planning_interface::MoveGroup& left_group,
    moveit::planning_interface::MoveGroup& right_group)
    : nh_(),
      left_group_(left_group),
      right_group_(right_group),
      arms_group_("arms"),
      planning_frame_(left_group_.getPlanningFrame()),
      left_gripper_(rapid::pr2::Gripper::Left()),
      right_gripper_(rapid::pr2::Gripper::Right()),
      left_traj_pub_(nh_.advertise<moveit_msgs::DisplayTrajectory>(
          "program_executor/left_arm_traj", 1, true)),
      right_traj_pub_(nh_.advertise<moveit_msgs::DisplayTrajectory>(
          "program_executor/right_arm_traj", 1, true)),
      gripper_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "program_executor/grippers", 10)),
      slice_pub_(nh_.advertise<msgs::ProgramSlices>("program_executor/slices",
                                                    1, true)),
      tf_listener_() {}

void ProgramExecutor::Init() {
  ROS_INFO("Using planning frame: %s", planning_frame_.c_str());
  left_group_.setPlannerId("RRTConnectkConfigDefault");
  right_group_.setPlannerId("RRTConnectkConfigDefault");
}

void ProgramExecutor::Execute(
    const msgs::Program& program,
    const std::map<std::string, msgs::ObjectState>& object_states) {
  // Split into left and right steps
  std::vector<msgs::Step> left_steps_raw;
  std::vector<msgs::Step> right_steps_raw;
  for (size_t i = 0; i < program.steps.size(); ++i) {
    const msgs::Step step = program.steps[i];
    if (step.arm == msgs::Step::LEFT) {
      left_steps_raw.push_back(step);
    } else if (step.arm == msgs::Step::RIGHT) {
      right_steps_raw.push_back(step);
    } else {
      ROS_ASSERT(false);
    }
  }

  ROS_INFO("Planning steps");
  std::vector<PlannedStep> left_steps =
      PlanSteps(left_steps_raw, object_states, left_group_);
  std::vector<PlannedStep> right_steps =
      PlanSteps(right_steps_raw, object_states, right_group_);

  // DEBUG
  bool is_valid = true;
  for (size_t i = 0; i < left_steps.size(); ++i) {
    const PlannedStep& step = left_steps[i];
    if (!IsValidTrajectory(step.traj)) {
      ROS_ERROR("Step %zu has invalid trajectory!", i);
      is_valid = false;
    }
  }
  if (is_valid) {
    ROS_INFO("Validated planned steps.");
  } else {
    ROS_ERROR("Planned steps invalid.");
  }

  msgs::ProgramSlices slices;
  slices.slices = SliceProgram(left_steps, right_steps);
  slice_pub_.publish(slices);
  ROS_INFO("Generated slices");

  // DEBUG
  is_valid = true;
  for (size_t i = 0; i < slices.slices.size(); ++i) {
    const msgs::ProgramSlice& slice = slices.slices[i];
    if (!IsValidTrajectory(slice.left_traj)) {
      ROS_ERROR("Slice %zu: left traj is invalid!", i);
      is_valid = false;
    }
    if (!IsValidTrajectory(slice.right_traj)) {
      ROS_ERROR("Slice %zu: right traj is invalid!", i);
      is_valid = false;
    }
  }
  if (is_valid) {
    ROS_INFO("Validated initial slices");
  } else {
    ROS_ERROR("Initial slices are invalid!");
  }

  msgs::ProgramSlices retimed_slices;
  retimed_slices.slices = RetimeSlices(slices.slices);
  slice_pub_.publish(retimed_slices);
  ROS_INFO("Done retiming slices.");

  ROS_INFO("Waiting for trigger to start execution...");
  ros::topic::waitForMessage<std_msgs::Bool>("trigger");

  bool is_sim = rapid::GetBoolParamOrThrow("use_sim_time");
  const double kGraspForce = is_sim ? -1 : 50;

  for (size_t i = 0; i < retimed_slices.slices.size(); ++i) {
    ProgramSlice& slice = retimed_slices.slices[i];
    if (slice.left_traj.points.size() == 0) {
      ROS_ASSERT(slice.is_left_closing ^ slice.is_left_opening);
      if (slice.is_left_closing) {
        slice.left_traj =
            GetNonMovingTrajectory(left_group_, ros::Duration(kGraspDuration));
      } else {
        slice.left_traj = GetNonMovingTrajectory(
            left_group_, ros::Duration(kUngraspDuration));
      }
    }
    if (slice.right_traj.points.size() == 0) {
      ROS_ASSERT(slice.is_right_closing ^ slice.is_right_opening);
      if (slice.is_right_closing) {
        slice.right_traj =
            GetNonMovingTrajectory(right_group_, ros::Duration(kGraspDuration));
      } else {
        slice.right_traj = GetNonMovingTrajectory(
            right_group_, ros::Duration(kUngraspDuration));
      }
    }

    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_.joint_trajectory =
        MergeTrajectories(slice.left_traj, slice.right_traj);

    // Remove all accelerations from the trajectory. This could possibly lead to
    // smoother executions.
    for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size();
         ++i) {
      JointTrajectoryPoint& pt = plan.trajectory_.joint_trajectory.points[i];
      pt.accelerations.clear();
    }

    // Execute slice
    if (slice.is_left_closing) {
      left_gripper_.StartClosing(kGraspForce);
    } else if (slice.is_left_opening) {
      left_gripper_.StartOpening();
    }
    if (slice.is_right_closing) {
      right_gripper_.StartClosing(kGraspForce);
    } else if (slice.is_right_opening) {
      right_gripper_.StartOpening();
    }
    arms_group_.execute(plan);
    if (slice.is_left_closing || slice.is_left_opening) {
      while (ros::ok() && !left_gripper_.IsDone()) {
        ros::spinOnce();
      }
    }
    if (slice.is_right_closing || slice.is_right_opening) {
      while (ros::ok() && !right_gripper_.IsDone()) {
        ros::spinOnce();
      }
    }
    double pause_duration;
    ros::param::param("pause_duration", pause_duration, 0.5);
    ros::Duration(pause_duration).sleep();
  }
}

std::string ProgramExecutor::planning_frame() const { return planning_frame_; }

std::vector<ProgramSlice> ProgramExecutor::RetimeSlices(
    const std::vector<ProgramSlice>& slices) {
  std::vector<ProgramSlice> retimed_slices;
  moveit::core::RobotStatePtr state = arms_group_.getCurrentState();
  robot_model::RobotModelConstPtr robot_model = arms_group_.getRobotModel();

  // MoveIt structures that are reused in the loop
  trajectory_processing::IterativeParabolicTimeParameterization retimer;
  robot_trajectory::RobotTrajectory left_traj(robot_model, "left_arm");
  robot_trajectory::RobotTrajectory right_traj(robot_model, "right_arm");

  for (size_t i = 0; i < slices.size(); ++i) {
    const ProgramSlice& slice = slices[i];
    ProgramSlice retimed_slice = slice;
    if (slice.left_traj.points.size() > 0) {
      left_traj.setRobotTrajectoryMsg(*state, slice.left_traj);
      retimer.computeTimeStamps(left_traj);
      moveit_msgs::RobotTrajectory msg;
      left_traj.getRobotTrajectoryMsg(msg);
      retimed_slice.left_traj = msg.joint_trajectory;
    }
    if (slice.right_traj.points.size() > 0) {
      right_traj.setRobotTrajectoryMsg(*state, slice.right_traj);
      retimer.computeTimeStamps(right_traj);
      moveit_msgs::RobotTrajectory msg;
      right_traj.getRobotTrajectoryMsg(msg);
      retimed_slice.right_traj = msg.joint_trajectory;
    }
    retimed_slices.push_back(retimed_slice);
  }
  return retimed_slices;
}

std::vector<ProgramSlice> SliceProgram(
    const std::vector<PlannedStep>& left_steps,
    const std::vector<PlannedStep>& right_steps) {
  std::vector<ProgramSlice> slices;

  size_t left_i = 0;
  size_t right_i = 0;

  ros::Time prev_time(0);
  ros::Time current_time(0);

  const ProgramSlice kBlankSlice;
  ProgramSlice current_slice;
  while (ros::ok() &&
         (left_i < left_steps.size() || right_i < right_steps.size())) {
    // Get start and end times
    ros::Time left_start(0);
    ros::Time left_end(0);
    ros::Time right_start(0);
    ros::Time right_end(0);
    std::vector<ros::Time> event_times;
    optional<PlannedStep> left_step = boost::none;
    if (left_i < left_steps.size()) {
      left_step = left_steps[left_i];
      left_start = left_step->traj.header.stamp;
      left_end = left_start + left_step->traj.points.back().time_from_start;
      event_times.push_back(left_start);
      event_times.push_back(left_end);
    }
    optional<PlannedStep> right_step = boost::none;
    if (right_i < right_steps.size()) {
      right_step = right_steps[right_i];
      right_start = right_step->traj.header.stamp;
      right_end = right_start + right_step->traj.points.back().time_from_start;
      event_times.push_back(right_start);
      event_times.push_back(right_end);
    }
    ROS_ASSERT(!(left_start.isZero() && right_start.isZero()));

    // Determine which event to process next (the first event whose time is
    // greater than prev_time).
    ros::Time best_time(0);
    for (size_t i = 0; i < event_times.size(); ++i) {
      ros::Time time = event_times[i];
      if (time > prev_time) {
        if (best_time.isZero() || time < best_time) {
          best_time = time;
        }
      }
    }
    current_time = best_time;

    if (!prev_time.isZero() && !current_time.isZero()) {
      // Get left slice from prev_time to current_time
      if (left_step) {
        current_slice.is_left_closing = left_step->is_closing;
        current_slice.is_left_opening = left_step->is_opening;
        current_slice.left_traj = left_step->GetTraj(prev_time, current_time);
        if (!IsValidTrajectory(current_slice.left_traj)) {
          ROS_ERROR("left slice is invalid");
        }
      }
      // Get right slice from prev_time to current_time
      if (right_step) {
        current_slice.is_right_closing = right_step->is_closing;
        current_slice.is_right_opening = right_step->is_opening;
        current_slice.right_traj = right_step->GetTraj(prev_time, current_time);
        if (!IsValidTrajectory(current_slice.right_traj)) {
          ROS_ERROR("right slice is invalid");
        }
      }

      if (!IsSliceEmpty(current_slice)) {
        slices.push_back(current_slice);
        current_slice = kBlankSlice;
      }
    }
    if (current_time == left_end) {
      ++left_i;
    }
    if (current_time == right_end) {
      ++right_i;
    }
    prev_time = current_time;
  }
  return slices;
}

std::vector<Pose> ComputeGraspTrajectory(const std::vector<Pose>& ee_trajectory,
                                         const Pose& current_obj_pose) {
  std::vector<Pose> gripper_traj(ee_trajectory.size());
  if (ee_trajectory.size() == 0) {
    return gripper_traj;
  }
  tg::Graph graph;
  graph.Add("object", tg::RefFrame("planning"), current_obj_pose);
  for (size_t i = 0; i < ee_trajectory.size(); ++i) {
    const Pose& ee_pose = ee_trajectory[i];
    tg::Transform ee_in_planning;
    graph.DescribePose(ee_pose, tg::Source("object"), tg::Target("planning"),
                       &ee_in_planning);
    gripper_traj[i] = ee_in_planning.pose();
  }
  return gripper_traj;
}

ros::Duration ComputeTrajectoryTime(const JointTrajectory& traj) {
  if (traj.points.size() > 0) {
    return traj.points.back().time_from_start;
  } else {
    return ros::Duration(0);
  }
}

std::vector<PlannedStep> PlanSteps(
    const std::vector<msgs::Step>& steps,
    const std::map<std::string, msgs::ObjectState>& object_states,
    moveit::planning_interface::MoveGroup& group) {
  std::vector<PlannedStep> result;
  robot_model::RobotModelConstPtr robot_model = group.getRobotModel();
  group.setStartStateToCurrentState();

  ros::Time start_time = ros::Time::now();
  robot_state::RobotStatePtr robot_state = group.getCurrentState();

  for (size_t i = 0; i < steps.size(); ++i) {
    const msgs::Step& step = steps[i];
    ros::Time prev_end(0);
    if (step.type == msgs::Step::GRASP) {
      std::vector<PlannedStep> planned_steps =
          PlanGraspStep(step, object_states, group, start_time, robot_state);
      result.insert(result.end(), planned_steps.begin(), planned_steps.end());
    } else if (step.type == msgs::Step::UNGRASP) {
      std::vector<PlannedStep> planned_steps =
          PlanUngraspStep(step, group, start_time, robot_state);
      result.insert(result.end(), planned_steps.begin(), planned_steps.end());
    } else if (step.type == msgs::Step::FOLLOW_TRAJECTORY) {
      PlannedStep planned_step = PlanFollowTrajectoryStep(
          step, object_states, group, start_time, robot_state);
      result.push_back(planned_step);
    } else if (step.type == msgs::Step::MOVE_TO_POSE) {
      PlannedStep planned_step = PlanMoveToPoseStep(step, object_states, group,
                                                    start_time, robot_state);
      result.push_back(planned_step);
    } else {
      ROS_ASSERT(false);
    }
  }
  return result;
}

std::vector<PlannedStep> PlanGraspStep(
    const task_perception_msgs::Step& step,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group, const ros::Time& start_time,
    robot_state::RobotStatePtr robot_state) {
  // Plan pre-grasp
  tg::Graph graph;
  graph.Add("current object", tg::RefFrame("planning"),
            object_states.at(step.object_state.name).pose);
  graph.Add("grasp", tg::RefFrame("current object"), step.ee_trajectory[0]);
  Pose pregrasp;
  pregrasp.orientation.w = 1;
  pregrasp.position.x = -0.1;
  tg::Transform pregrasp_in_planning;
  graph.DescribePose(pregrasp, tg::Source("grasp"), tg::Target("planning"),
                     &pregrasp_in_planning);

  group.setStartState(*robot_state);
  group.setPoseTarget(pregrasp_in_planning.pose());
  moveit::planning_interface::MoveGroup::Plan pregrasp_plan;
  group.plan(pregrasp_plan);
  ros::Duration pregrasp_duration =
      ComputeTrajectoryTime(pregrasp_plan.trajectory_.joint_trajectory);
  moveit::core::jointTrajPointToRobotState(
      pregrasp_plan.trajectory_.joint_trajectory,
      pregrasp_plan.trajectory_.joint_trajectory.points.size() - 1,
      *robot_state);

  // Plan movement to grasp pose
  tg::Transform grasp_in_planning;
  graph.ComputeDescription("grasp", tg::RefFrame("planning"),
                           &grasp_in_planning);
  group.setStartState(*robot_state);
  group.setPoseTarget(grasp_in_planning.pose());
  moveit::planning_interface::MoveGroup::Plan grasp_plan;
  group.plan(grasp_plan);
  ros::Duration grasp_duration =
      ComputeTrajectoryTime(grasp_plan.trajectory_.joint_trajectory);
  moveit::core::jointTrajPointToRobotState(
      grasp_plan.trajectory_.joint_trajectory,
      grasp_plan.trajectory_.joint_trajectory.points.size() - 1, *robot_state);

  std::vector<PlannedStep> result;
  // Add pregrasp step
  PlannedStep pregrasp_step;
  pregrasp_step.traj = pregrasp_plan.trajectory_.joint_trajectory;
  pregrasp_step.traj.header.stamp =
      start_time + step.start_time - grasp_duration - pregrasp_duration;
  result.push_back(pregrasp_step);

  // Add move-to-grasp step
  PlannedStep move_to_grasp;
  move_to_grasp.traj = grasp_plan.trajectory_.joint_trajectory;
  move_to_grasp.traj.header.stamp =
      start_time + step.start_time - grasp_duration;
  result.push_back(move_to_grasp);

  // Add grasp step (the trajectory is to hold still)
  PlannedStep grasp;
  grasp.traj.header.stamp = start_time + step.start_time;
  grasp.traj.joint_names = move_to_grasp.traj.joint_names;
  JointTrajectoryPoint end_pt = move_to_grasp.traj.points.back();
  end_pt.time_from_start = ros::Duration(kGraspDuration);
  grasp.traj.points.push_back(end_pt);
  // TODO: could scale the pre grasp trajectory in case we overrun into another
  // Step's allocated time.
  grasp.is_closing = true;
  result.push_back(grasp);
  return result;
}

std::vector<PlannedStep> PlanUngraspStep(
    const task_perception_msgs::Step& step,
    moveit::planning_interface::MoveGroup& group, const ros::Time& start_time,
    robot_state::RobotStatePtr start_state) {
  tg::Graph graph;
  graph.Add("current ee pose", tg::RefFrame("planning frame"),
            start_state->getGlobalLinkTransform(group.getEndEffectorLink()));
  graph.Add("post grasp", tg::RefFrame("current ee pose"),
            tg::Transform(tg::Position(-0.1, 0, 0), tg::Orientation()));
  tg::Transform post_grasp_pose;
  graph.ComputeDescription("post grasp", tg::RefFrame("planning frame"),
                           &post_grasp_pose);
  group.setStartState(*start_state);
  group.setPoseTarget(post_grasp_pose.pose());
  moveit::planning_interface::MoveGroup::Plan post_grasp_plan;
  group.plan(post_grasp_plan);

  // Update start state
  moveit::core::jointTrajPointToRobotState(
      post_grasp_plan.trajectory_.joint_trajectory,
      post_grasp_plan.trajectory_.joint_trajectory.points.size() - 1,
      *start_state);

  std::vector<PlannedStep> result;

  // Add ungrasp
  PlannedStep ungrasp;
  ungrasp.traj.header.stamp = start_time + step.start_time;
  ungrasp.traj.joint_names =
      post_grasp_plan.trajectory_.joint_trajectory.joint_names;
  JointTrajectoryPoint start_pt =
      post_grasp_plan.trajectory_.joint_trajectory.points.front();
  start_pt.time_from_start = ros::Duration(kUngraspDuration);
  ungrasp.traj.points.push_back(start_pt);

  ungrasp.is_opening = true;
  result.push_back(ungrasp);

  // Add post grasp
  PlannedStep post_grasp;
  post_grasp.traj = post_grasp_plan.trajectory_.joint_trajectory;
  post_grasp.traj.header.stamp =
      start_time + step.start_time + ros::Duration(kUngraspDuration);

  result.push_back(post_grasp);
  // TODO: could scale the post grasp trajectory in case we overrun into another
  // Step's allocated time.

  return result;
}

PlannedStep PlanFollowTrajectoryStep(
    const task_perception_msgs::Step& step,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group, const ros::Time& start_time,
    robot_state::RobotStatePtr start_state) {
  PlannedStep result;
  moveit_msgs::MoveItErrorCodes error_code;
  moveit_msgs::RobotTrajectory planned_traj;

  double jump_threshold;
  ros::param::param("jump_threshold", jump_threshold, 1.6);
  const bool kAvoidCollisions = true;

  if (step.ee_trajectory.size() == 0) {
    PlannedStep blank;
    return blank;
  }

  // Plan trajectory
  std::vector<Pose> pose_trajectory;
  Pose current_obj = object_states.at(step.object_state.name).pose;
  pose_trajectory = ComputeGraspTrajectory(step.ee_trajectory, current_obj);
  group.setStartState(*start_state);

  double fraction =
      group.computeCartesianPath(pose_trajectory, 0.01, jump_threshold,
                                 planned_traj, kAvoidCollisions, &error_code);
  if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR("Failed to plan arm trajectory. MoveIt error %d", error_code.val);
  } else if (step.ee_trajectory.size() > 0 && fraction < 1) {
    ROS_ERROR("Planned %f%% of arm trajectory", fraction * 100);
  } else {
    ROS_INFO("Planned %f%% of arm trajectory", fraction * 100);
  }

  // Update start state
  moveit::core::jointTrajPointToRobotState(
      planned_traj.joint_trajectory,
      planned_traj.joint_trajectory.points.size() - 1, *start_state);

  // Scale the trajectory so that it takes the same amount of time as in the
  // human demonstration.
  ros::Duration intended_duration = step.times_from_start.back();
  ros::Duration planned_duration =
      planned_traj.joint_trajectory.points.back().time_from_start;
  double scale_factor = intended_duration.toSec() / planned_duration.toSec();
  for (size_t i = 0; i < planned_traj.joint_trajectory.points.size(); ++i) {
    planned_traj.joint_trajectory.points[i].time_from_start *= scale_factor;
  }

  result.traj = planned_traj.joint_trajectory;
  result.traj.header.stamp = start_time + step.start_time;

  return result;
}

PlannedStep PlanMoveToPoseStep(
    const task_perception_msgs::Step& step,
    const std::map<std::string, task_perception_msgs::ObjectState>&
        object_states,
    moveit::planning_interface::MoveGroup& group, const ros::Time& start_time,
    robot_state::RobotStatePtr start_state) {
  tg::Graph graph;
  graph.Add("current object", tg::RefFrame("planning"),
            object_states.at(step.object_state.name).pose);
  graph.Add("goal pose", tg::RefFrame("current object"), step.ee_trajectory[0]);
  tg::Transform goal_in_planning;
  graph.ComputeDescription("goal pose", tg::RefFrame("planning"),
                           &goal_in_planning);

  group.setStartState(*start_state);
  group.setPoseTarget(goal_in_planning.pose());

  moveit::planning_interface::MoveGroup::Plan plan;
  group.plan(plan);
  moveit::core::jointTrajPointToRobotState(
      plan.trajectory_.joint_trajectory,
      plan.trajectory_.joint_trajectory.points.size() - 1, *start_state);

  // Scale the trajectory so that it takes the same amount of time as was
  // specified by the human demonstration. Generally, the human demonstration
  // will be faster than the robot, so we are speeding up the trajectory here.
  // Later, we will slice the trajectory to ensure synchronization with the
  // other hand. Each slice is then re-timed to work properly on the robot.
  ros::Duration intended_duration = step.times_from_start[0];
  ros::Duration planned_time =
      ComputeTrajectoryTime(plan.trajectory_.joint_trajectory);
  if (planned_time.isZero()) {
    ROS_ERROR("No trajectory generated for move-to-pose step!");
    PlannedStep result;
    return result;
  }

  double scale_factor = intended_duration.toSec() / planned_time.toSec();
  for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); ++i) {
    plan.trajectory_.joint_trajectory.points[i].time_from_start *= scale_factor;
  }

  PlannedStep result;
  result.traj = plan.trajectory_.joint_trajectory;
  result.traj.header.stamp = start_time + step.start_time;
  return result;
}

bool IsValidTrajectory(const JointTrajectory& traj) {
  // Verify timestamps are monotonically increasing
  for (size_t i = 0; i + 1 < traj.points.size(); ++i) {
    const JointTrajectoryPoint& pt = traj.points[i];
    const JointTrajectoryPoint& next_pt = traj.points[i + 1];
    if (next_pt.time_from_start < pt.time_from_start) {
      return false;
    }
  }

  // Verify joint_names matches positions in length
  if (traj.points.size() > 0) {
    if (traj.joint_names.size() != traj.points[0].positions.size()) {
      return false;
    }
  }
  return true;
}
}  // namespace pbi
