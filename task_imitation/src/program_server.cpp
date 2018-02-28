#include "task_imitation/program_server.h"

#include <limits.h>
#include <map>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_state/conversions.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/Program.h"
#include "task_utils/bag_utils.h"
#include "task_utils/pr2_gripper_viz.h"
#include "tf/transform_listener.h"
#include "transform_graph/graph.h"

#include "task_imitation/bimanual_manipulation.h"
#include "task_imitation/program_generator.h"
#include "task_imitation/program_iterator.h"
#include "task_imitation/program_slice.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using boost::optional;
using geometry_msgs::Pose;

namespace pbi {
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
    if (step.action_type == msgs::Step::GRASP) {
      // If this slice already has a grasp, then submit the slice and reset.
      if (current_slice.grasp.action_type != "") {
        current_slice.FixTrajectories();
        slices.push_back(current_slice);
        current_slice.Reset();
      }
      current_slice.grasp = step;
      it->Advance();
    } else if (step.action_type == msgs::Step::FOLLOW_TRAJECTORY) {
      // Initialize trajectory message if needed.
      if (traj_step->ee_trajectory.size() == 0) {
        traj_step->start_time = step.start_time;
        traj_step->arm = step.arm;
        traj_step->action_type = step.action_type;
        traj_step->object_state = step.object_state;
      }
      optional<std::pair<Pose, ros::Duration> > pt = it->trajectory_point();
      ROS_ASSERT(pt);
      traj_step->ee_trajectory.push_back(pt->first);
      traj_step->times_from_start.push_back(pt->second);
      it->Advance();
    } else if (step.action_type == msgs::Step::UNGRASP) {
      current_slice.ungrasp = step;
      current_slice.FixTrajectories();
      slices.push_back(current_slice);
      current_slice.Reset();
      it->Advance();
    }
  }

  return slices;
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

ProgramServer::ProgramServer(const ros::ServiceClient& db_client)
    : db_client_(db_client),
      left_group_("left_arm"),
      right_group_("right_arm"),
      arms_group_("arms"),
      nh_(),
      action_server_(
          nh_, "imitate_demo",
          boost::bind(&pbi::ProgramServer::ExecuteImitation, this, _1), false),
      initialize_object_("initialize_object"),
      planning_frame_(left_group_.getPlanningFrame()),
      left_traj_pub_(nh_.advertise<moveit_msgs::DisplayTrajectory>(
          "program_executor/left_arm_traj", 1, true)),
      right_traj_pub_(nh_.advertise<moveit_msgs::DisplayTrajectory>(
          "program_executor/right_arm_traj", 1, true)),
      left_gripper_(pr2_actions::Gripper::Left()),
      right_gripper_(pr2_actions::Gripper::Right()),
      tf_listener_(),
      gripper_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "program_executor/grippers", 10)) {}

void ProgramServer::Start() {
  ROS_INFO("Using planning frame: %s", planning_frame_.c_str());
  action_server_.start();
  while (ros::ok() && !initialize_object_.waitForServer(ros::Duration(2.0))) {
    ROS_WARN("Waiting for object initializer action.");
  }
  left_group_.setPlannerId("RRTConnectkConfigDefault");
  right_group_.setPlannerId("RRTConnectkConfigDefault");
}

void ProgramServer::ExecuteImitation(
    const msgs::ImitateDemoGoalConstPtr& goal) {
  msgs::GetDemoStatesRequest get_states_req;
  get_states_req.name = GetNameFromBagPath(goal->bag_path);
  msgs::GetDemoStatesResponse get_states_res;
  db_client_.call(get_states_req, get_states_res);
  if (get_states_res.error != "") {
    ROS_ERROR("%s", get_states_res.error.c_str());
    msgs::ImitateDemoResult result;
    result.error = get_states_res.error;
    action_server_.setAborted(result, get_states_res.error);
    return;
  }

  // Generate program and slices
  const msgs::DemoStates& demo_states = get_states_res.demo_states;
  ProgramGenerator generator;
  for (size_t i = 0; i < demo_states.demo_states.size(); ++i) {
    generator.Step(demo_states.demo_states[i]);
  }
  msgs::Program program = generator.program();
  std::map<std::string, msgs::ObjectState> object_states =
      GetObjectPoses(program);
  ROS_INFO("All object states initialized.");
  std::vector<Slice> slices = ComputeSlices(program, object_states);
  ROS_INFO("Generated slices");

  // Execute each slice
  bool is_sim;
  ros::param::param("use_sim_time", is_sim, false);
  double grasp_force = is_sim ? -1 : 50;

  for (size_t i = 0; i < slices.size(); ++i) {
    const Slice& slice = slices[i];

    ROS_INFO("Slice %ld: grasp: %s, left pts: %ld, right pts: %ld, ungrasp: %s",
             i, slice.grasp.arm.c_str(), slice.left_traj.ee_trajectory.size(),
             slice.right_traj.ee_trajectory.size(), slice.ungrasp.arm.c_str());

    // Execute grasp, if applicable
    if (slice.grasp.arm != "") {
      Pose grasp = slice.grasp.ee_trajectory[0];
      tg::Graph graph;
      graph.Add("grasp", tg::RefFrame(planning_frame_), grasp);
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
      }
    }

    const bool kAvoidCollisions = true;
    moveit_msgs::MoveItErrorCodes error_code;
    moveit_msgs::RobotTrajectory left_traj;

    // Plan trajectory
    double jump_threshold;
    ros::param::param("jump_threshold", jump_threshold, 1.6);
    double left_fraction = left_group_.computeCartesianPath(
        SampleTrajectory(slice.left_traj.ee_trajectory), 0.01, jump_threshold,
        left_traj, kAvoidCollisions, &error_code);
    if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_ERROR("Failed to plan left arm trajectory. MoveIt error %d",
                error_code.val);
    } else if (slice.left_traj.ee_trajectory.size() > 0 && left_fraction < 1) {
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
    double right_fraction = right_group_.computeCartesianPath(
        SampleTrajectory(slice.right_traj.ee_trajectory), 0.01, jump_threshold,
        right_traj, kAvoidCollisions, &error_code);
    if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_ERROR("Failed to plan right arm trajectory. MoveIt error %d",
                error_code.val);
    } else if (slice.right_traj.ee_trajectory.size() > 0 &&
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

  msgs::ImitateDemoResult result;
  action_server_.setSucceeded(result);
}

std::map<std::string, msgs::ObjectState> ProgramServer::GetObjectPoses(
    const msgs::Program& program) {
  // This models the assumption that each demonstration only interacts with an
  // object once.We could / should allow the robot to interact with an object
  // more than once, but we need to continuosly track the objects in that
  // case.
  std::map<std::string, msgs::ObjectState> object_states;
  for (size_t i = 0; i < program.steps.size(); ++i) {
    const msgs::Step& step = program.steps[i];
    if (step.action_type == msgs::Step::GRASP) {
      object_states[step.object_state.name] = step.object_state;
    }
  }

  for (std::map<std::string, msgs::ObjectState>::iterator it =
           object_states.begin();
       it != object_states.end(); ++it) {
    dbot_ros_msgs::InitializeObjectGoal init_goal;
    init_goal.frame_id = planning_frame_;
    init_goal.mesh_name = it->second.mesh_name;
    init_goal.initial_pose.orientation.w = 1;
    init_goal.initial_pose.position.x = 1;
    init_goal.initial_pose.position.z = 1;
    initialize_object_.sendGoal(init_goal);
    while (ros::ok() && !initialize_object_.getState().isDone()) {
      ros::spinOnce();
    }
    dbot_ros_msgs::InitializeObjectResultConstPtr init_result =
        initialize_object_.getResult();
    it->second.pose = init_result->pose;
    ROS_INFO_STREAM("Initialized pose for object: \""
                    << it->first << "\" at pose " << it->second.pose);
  }

  return object_states;
}

std::vector<Slice> ProgramServer::ComputeSlices(
    const msgs::Program& program,
    const std::vector<msgs::ObjectState>& object_states) {
  Pr2GripperViz gripper_viz;
  gripper_viz.set_frame_id(planning_frame_);

  // Transform all poses into planning frame
  // Note: this changes the semantics of a Program. Normally, the trajectory
  // poses are relative to the object. From here on, they are relative to the
  // planning frame, regardless of where the object is.
  tg::Graph graph;
  for (std::map<std::string, msgs::ObjectState>::const_iterator it =
           object_states.begin();
       it != object_states.end(); ++it) {
    const msgs::ObjectState& os = it->second;
    graph.Add(os.name, tg::RefFrame(planning_frame_), os.pose);
  }
  for (size_t step_i = 0; step_i < program.steps.size(); ++step_i) {
    msgs::Step& step = program.steps[step_i];
    if (step.action_type == msgs::Step::GRASP ||
        step.action_type == msgs::Step::FOLLOW_TRAJECTORY) {
      const std::string& object_name = step.object_state.name;
      for (size_t traj_i = 0; traj_i < step.ee_trajectory.size(); ++traj_i) {
        const Pose& pose = step.ee_trajectory[traj_i];
        tg::Transform pose_in_planning;
        bool success =
            graph.DescribePose(pose, tg::Source(object_name),
                               tg::Target(planning_frame_), &pose_in_planning);
        ROS_ASSERT(success);
        step.ee_trajectory[traj_i] = pose_in_planning.pose();
        gripper_viz.set_pose(step.ee_trajectory[traj_i]);
        gripper_pub_.publish(gripper_viz.markers("program"));
        ros::Duration(0.033).sleep();
      }
    }
  }
  return SliceProgram(program);
}
}  // namespace pbi
