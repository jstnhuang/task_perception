#include "task_imitation/program_executor.h"

#include <map>
#include <vector>

#include "boost/optional.hpp"
#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_state/conversions.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "pr2_actions/gripper.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/Step.h"
#include "task_utils/pr2_gripper_viz.h"
#include "transform_graph/graph.h"

#include "task_imitation/bimanual_manipulation.h"
#include "task_imitation/grasp_planning_context.h"
#include "task_imitation/program_iterator.h"
#include "task_imitation/program_slice.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using boost::optional;
using geometry_msgs::Pose;

namespace pbi {
ProgramExecutor::ProgramExecutor()
    : nh_(),
      left_group_("left_arm"),
      right_group_("right_arm"),
      arms_group_("arms"),
      planning_frame_(left_group_.getPlanningFrame()),
      left_gripper_(pr2_actions::Gripper::Left()),
      right_gripper_(pr2_actions::Gripper::Right()),
      left_traj_pub_(nh_.advertise<moveit_msgs::DisplayTrajectory>(
          "program_executor/left_arm_traj", 1, true)),
      right_traj_pub_(nh_.advertise<moveit_msgs::DisplayTrajectory>(
          "program_executor/right_arm_traj", 1, true)),
      gripper_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "program_executor/grippers", 10)),
      tf_listener_() {}

void ProgramExecutor::Init() {
  ROS_INFO("Using planning frame: %s", planning_frame_.c_str());
  left_group_.setPlannerId("RRTConnectkConfigDefault");
  right_group_.setPlannerId("RRTConnectkConfigDefault");
}

void ProgramExecutor::Execute(
    const msgs::Program& program,
    const std::map<std::string, msgs::ObjectState>& object_states) {
  std::vector<Slice> slices = SliceProgram(program);
  ROS_INFO("Generated slices");

  // Execute each slice
  bool is_sim;
  ros::param::param("use_sim_time", is_sim, false);
  double grasp_force = is_sim ? -1 : 50;

  Pose current_left_grasp_in_obj;
  Pose current_right_grasp_in_obj;

  for (size_t i = 0; i < slices.size(); ++i) {
    const Slice& slice = slices[i];

    ROS_INFO(
        "Slice %ld: grasp: %s, left pts: %ld, right pts: %ld, ungrasp: %s", i,
        slice.grasp.arm.c_str(), slice.left_traj.object_trajectory.size(),
        slice.right_traj.object_trajectory.size(), slice.ungrasp.arm.c_str());

    // Execute grasp, if applicable
    if (slice.grasp.arm != "") {
      const msgs::ObjectState& demo_object_state = slice.grasp.object_state;
      GraspPlanningContext context(
          slice.grasp.wrist_pose, planning_frame_, demo_object_state.name,
          demo_object_state.mesh_name, demo_object_state.pose);
      Pose demo_grasp = grasp_planner_.Plan(context);
      // TODO: the grasp planner gives the grasp at the time of the
      // demonstration.
      // We need to adapt the grasp to the current object position.
      tg::Graph graph;
      graph.Add("demo grasp", tg::RefFrame(planning_frame_), demo_grasp);
      graph.Add("demo object", tg::RefFrame(planning_frame_),
                demo_object_state.pose);
      tg::Transform grasp_in_object;
      graph.ComputeDescription("demo grasp", tg::RefFrame("demo object"),
                               &grasp_in_object);
      graph.Add("current object", tg::RefFrame(planning_frame_),
                object_states.at(demo_object_state.name).pose);
      tg::Transform grasp_in_planning_frame;
      graph.DescribePose(grasp_in_object, tg::Source("current object"),
                         tg::Target(planning_frame_), &grasp_in_planning_frame);
      Pose grasp = grasp_in_planning_frame.pose();

      graph.Add("grasp", tg::RefFrame(planning_frame_),
                grasp_in_planning_frame);
      Pose pregrasp;
      pregrasp.orientation.w = 1;
      pregrasp.position.x = -0.10;
      tg::Transform pregrasp_in_planning;
      graph.DescribePose(pregrasp, tg::Source("grasp"),
                         tg::Target(planning_frame_), &pregrasp_in_planning);

      if (slice.grasp.arm == msgs::Step::LEFT) {
        left_gripper_.StartOpening();
        left_group_.setPoseTarget(pregrasp_in_planning.pose());
        left_group_.move();
        while (!left_gripper_.IsDone() && ros::ok()) {
          ros::spinOnce();
        }
        left_group_.setPoseTarget(grasp);
        left_group_.move();
        left_gripper_.StartClosing(grasp_force);
        current_left_grasp_in_obj = grasp_in_object.pose();
        while (!left_gripper_.IsDone() && ros::ok()) {
          ros::spinOnce();
        }
      } else if (slice.grasp.arm == msgs::Step::RIGHT) {
        right_gripper_.StartOpening();
        right_group_.setPoseTarget(pregrasp_in_planning.pose());
        right_group_.move();
        while (!right_gripper_.IsDone() && ros::ok()) {
          ros::spinOnce();
        }
        right_group_.setPoseTarget(grasp);
        right_group_.move();
        right_gripper_.StartClosing(grasp_force);
        while (!right_gripper_.IsDone() && ros::ok()) {
          ros::spinOnce();
        }
        current_right_grasp_in_obj = grasp_in_object.pose();
      }
    }

    const bool kAvoidCollisions = true;
    moveit_msgs::MoveItErrorCodes error_code;
    moveit_msgs::RobotTrajectory left_traj;

    // Plan trajectory
    double jump_threshold;
    ros::param::param("jump_threshold", jump_threshold, 1.6);
    std::vector<Pose> left_ee_trajectory = ComputeGraspTrajectory(
        current_left_grasp_in_obj, slice.left_traj.object_trajectory);
    double left_fraction = left_group_.computeCartesianPath(
        SampleTrajectory(left_ee_trajectory), 0.01, jump_threshold, left_traj,
        kAvoidCollisions, &error_code);
    if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_ERROR("Failed to plan left arm trajectory. MoveIt error %d",
                error_code.val);
    } else if (slice.left_traj.object_trajectory.size() > 0 &&
               left_fraction < 1) {
      ROS_ERROR("Planned %f%% of left arm trajectory", left_fraction * 100);
    } else {
      ROS_INFO("Planned %f%% of left arm trajectory", left_fraction * 100);
    }
    if (left_traj.joint_trajectory.points.size() == 0) {
      left_traj.joint_trajectory = GetNonMovingTrajectory(left_group_);
    }
    moveit_msgs::DisplayTrajectory left_display;
    moveit::core::robotStateToRobotStateMsg(*left_group_.getCurrentState(),
                                            left_display.trajectory_start);
    left_display.trajectory.push_back(left_traj);
    left_traj_pub_.publish(left_display);

    moveit_msgs::RobotTrajectory right_traj;
    std::vector<Pose> right_ee_trajectory = ComputeGraspTrajectory(
        current_right_grasp_in_obj, slice.right_traj.object_trajectory);
    double right_fraction = right_group_.computeCartesianPath(
        SampleTrajectory(right_ee_trajectory), 0.01, jump_threshold, right_traj,
        kAvoidCollisions, &error_code);
    if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_ERROR("Failed to plan right arm trajectory. MoveIt error %d",
                error_code.val);
    } else if (slice.right_traj.object_trajectory.size() > 0 &&
               right_fraction < 1) {
      ROS_ERROR("Planned %f%% of right arm trajectory", right_fraction * 100);
    } else {
      ROS_INFO("Planned %f%% of right arm trajectory", right_fraction * 100);
    }
    if (right_traj.joint_trajectory.points.size() == 0) {
      right_traj.joint_trajectory = GetNonMovingTrajectory(right_group_);
    }
    moveit_msgs::DisplayTrajectory right_display;
    moveit::core::robotStateToRobotStateMsg(*right_group_.getCurrentState(),
                                            right_display.trajectory_start);
    right_display.trajectory.push_back(right_traj);
    right_traj_pub_.publish(right_display);

    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = MergeTrajectories(left_traj, right_traj);
    arms_group_.execute(plan);

    // Execute ungrasp, if applicable
    if (slice.ungrasp.arm != "") {
      std::string link;
      Pose wrist_pose;
      if (slice.ungrasp.arm == msgs::Step::LEFT) {
        link = "l_wrist_roll_link";
      } else if (slice.ungrasp.arm == msgs::Step::RIGHT) {
        link = "r_wrist_roll_link";
      }
      tf::StampedTransform wrist_tf;
      ros::Time now = ros::Time::now();
      tf_listener_.waitForTransform(planning_frame_, link, now,
                                    ros::Duration(5.0));
      tf_listener_.lookupTransform(planning_frame_, link, now, wrist_tf);
      tg::Transform wrist_tg(wrist_tf);

      tg::Graph graph;
      graph.Add("current", tg::RefFrame(planning_frame_), wrist_tf);
      Pose release;
      release.orientation.w = 1;
      release.position.x = -0.08;
      tg::Transform release_in_planning;
      graph.DescribePose(release, tg::Source("current"),
                         tg::Target(planning_frame_), &release_in_planning);

      if (slice.ungrasp.arm == msgs::Step::LEFT) {
        left_gripper_.StartOpening();
        while (!left_gripper_.IsDone() && ros::ok()) {
          ros::spinOnce();
        }
        left_group_.setPoseTarget(release_in_planning.pose());
        left_group_.move();
      } else if (slice.ungrasp.arm == msgs::Step::RIGHT) {
        right_gripper_.StartOpening();
        while (!right_gripper_.IsDone() && ros::ok()) {
          ros::spinOnce();
        }
        right_group_.setPoseTarget(release_in_planning.pose());
        right_group_.move();
      }
    }
  }
}

std::string ProgramExecutor::planning_frame() const { return planning_frame_; }

// std::vector<Slice> ProgramExecutor::ComputeSlices(
//    const msgs::Program& program,
//    const std::map<std::string, msgs::ObjectState>& object_states) {
//  Pr2GripperViz gripper_viz;
//  gripper_viz.set_frame_id(planning_frame_);
//
//  // Transform all poses into planning frame
//  // Note: this changes the semantics of a Step. Normally, the trajectory
//  // specifies the motion of the object. Now, they represent the motion of the
//  // end-effector.
//  tg::Graph graph;
//  for (std::map<std::string, msgs::ObjectState>::const_iterator it =
//           object_states.begin();
//       it != object_states.end(); ++it) {
//    const msgs::ObjectState& os = it->second;
//    graph.Add(os.name, tg::RefFrame(planning_frame_), os.pose);
//  }
//  for (size_t step_i = 0; step_i < program.steps.size(); ++step_i) {
//    msgs::Step step = program.steps[step_i];
//    if (step.type == msgs::Step::GRASP ||
//        step.type == msgs::Step::FOLLOW_TRAJECTORY) {
//      const std::string& object_name = step.object_name;
//      for (size_t traj_i = 0; traj_i < step.ee_trajectory.size(); ++traj_i) {
//        const Pose& pose = step.ee_trajectory[traj_i];
//        tg::Transform pose_in_planning;
//        bool success =
//            graph.DescribePose(pose, tg::Source(object_name),
//                               tg::Target(planning_frame_),
//                               &pose_in_planning);
//        ROS_ASSERT(success);
//        step.ee_trajectory[traj_i] = pose_in_planning.pose();
//        gripper_viz.set_pose(step.ee_trajectory[traj_i]);
//        gripper_pub_.publish(gripper_viz.markers("program"));
//        ros::Duration(0.033).sleep();
//      }
//    }
//  }
//  return SliceProgram(program);
//}

std::vector<Slice> SliceProgram(const msgs::Program& program) {
  // Split into left/right steps
  std::vector<msgs::Step> left_steps;
  std::vector<msgs::Step> right_steps;

  for (size_t i = 0; i < program.steps.size(); ++i) {
    const msgs::Step& step = program.steps[i];
    if (step.arm == msgs::Step::LEFT) {
      left_steps.push_back(step);
    } else if (step.arm == msgs::Step::RIGHT) {
      right_steps.push_back(step);
    }
  }

  // Iterate through left and right actions.
  ProgramIterator left_it(left_steps);
  ProgramIterator right_it(right_steps);
  left_it.Begin();
  right_it.Begin();
  std::vector<Slice> slices;
  Slice current_slice;
  while (!left_it.IsDone() || !right_it.IsDone()) {
    // Walk through the left and right steps in order of time.
    ProgramIterator* it;
    msgs::Step* traj_step;
    if (left_it.IsDone()) {
      it = &right_it;
      traj_step = &current_slice.right_traj;
    } else if (right_it.IsDone()) {
      it = &left_it;
      traj_step = &current_slice.left_traj;
    } else if (left_it.time() < right_it.time()) {
      it = &left_it;
      traj_step = &current_slice.left_traj;
    } else {
      it = &right_it;
      traj_step = &current_slice.right_traj;
    }

    const msgs::Step& step = it->step();
    if (step.type == msgs::Step::GRASP) {
      // If this slice already has a grasp, then submit the slice and reset.
      if (current_slice.grasp.type != "") {
        current_slice.FixTrajectories();
        slices.push_back(current_slice);
        current_slice.Reset();
      }
      current_slice.grasp = step;
      it->Advance();
    } else if (step.type == msgs::Step::FOLLOW_TRAJECTORY) {
      // Initialize trajectory message if needed.
      if (traj_step->object_trajectory.size() == 0) {
        traj_step->start_time = step.start_time;
        traj_step->arm = step.arm;
        traj_step->type = step.type;
      }
      optional<std::pair<Pose, ros::Duration> > pt = it->trajectory_point();
      ROS_ASSERT(pt);
      traj_step->object_trajectory.push_back(pt->first);
      traj_step->times_from_start.push_back(pt->second);
      it->Advance();
    } else if (step.type == msgs::Step::UNGRASP) {
      current_slice.ungrasp = step;
      current_slice.FixTrajectories();
      slices.push_back(current_slice);
      current_slice.Reset();
      it->Advance();
    }
  }

  return slices;
}

std::vector<Pose> ComputeGraspTrajectory(
    const Pose& grasp_in_obj, const std::vector<Pose>& object_trajectory) {
  std::vector<Pose> gripper_traj(object_trajectory.size());
  tg::Graph graph;
  graph.Add("grasp", tg::RefFrame("object"), grasp_in_obj);
  for (size_t i = 0; i < object_trajectory.size(); ++i) {
    const Pose& obj_pose = object_trajectory[i];
    graph.Add("object", tg::RefFrame("planning frame"), obj_pose);
    tg::Transform grasp_in_planning;
    graph.ComputeDescription("grasp", tg::RefFrame("planning frame"),
                             &grasp_in_planning);
    gripper_traj[i] = grasp_in_planning.pose();
  }
  return gripper_traj;
}

std::vector<Pose> SampleTrajectory(const std::vector<Pose>& traj) {
  std::vector<Pose> sampled;
  if (traj.size() == 0) {
    return traj;
  }
  sampled.push_back(traj[0]);

  int sample_every;
  ros::param::param("sample_every", sample_every, 9);
  for (size_t i = sample_every; i < traj.size() - 1; i += sample_every) {
    sampled.push_back(traj[i]);
  }

  sampled.push_back(traj.back());
  ROS_INFO("Sampled %ld poses out of %ld", sampled.size(), traj.size());
  return sampled;
}
}  // namespace pbi
