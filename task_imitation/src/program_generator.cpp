// Given a demonstration as a sequence of DemoStates, generates a program to
// imitate the demonstration.
#include "task_imitation/program_generator.h"

#include <string>

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/HandState.h"
#include "task_perception_msgs/ObjectState.h"
#include "task_perception_msgs/Program.h"
#include "task_perception_msgs/Step.h"
#include "transform_graph/graph.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;

namespace pbi {
const double ProgramGenerator::kGraspDuration = 2;
const double ProgramGenerator::kUngraspDuration = 2;

ProgramGenerator::ProgramGenerator()
    : program_(), prev_state_(), start_time_(0) {}

void ProgramGenerator::Step(const msgs::DemoState& state) {
  ProcessContact(state, msgs::Step::LEFT);
  ProcessContact(state, msgs::Step::RIGHT);
  prev_state_ = state;
}

msgs::Program ProgramGenerator::program() const { return program_; }

void ProgramGenerator::ProcessContact(const msgs::DemoState& state,
                                      const std::string& arm_name) {
  msgs::HandState hand;
  msgs::HandState prev_hand;
  if (arm_name == msgs::Step::LEFT) {
    hand = state.left_hand;
    prev_hand = prev_state_.left_hand;
  } else if (arm_name == msgs::Step::RIGHT) {
    hand = state.right_hand;
    prev_hand = prev_state_.right_hand;
  }

  // If we start a contact, then add a grasp action and start a trajectory
  // If we are continuing a contact, then append to the trajectory
  // If we are ending a contact, then end the trajectory
  if (hand.current_action == msgs::HandState::GRASPING &&
      (prev_hand.current_action == "" ||
       prev_hand.current_action == msgs::HandState::NONE)) {
    if (program_.steps.size() == 0) {
      start_time_ = state.stamp;
    }

    msgs::Step grasp_step;

    // Compute start time
    msgs::Step* prev_step = GetMostRecentStep(arm_name);
    grasp_step.start_time = state.stamp - start_time_;
    if (prev_step != NULL) {
      ros::Duration prev_end = GetEndTime(*prev_step);
      if (prev_end > grasp_step.start_time) {
        ros::Duration dt = state.stamp - prev_state_.stamp;
        grasp_step.start_time = prev_end + dt;
      }
    }
    grasp_step.arm = arm_name;
    grasp_step.action_type = msgs::Step::GRASP;
    GetObjectState(state, hand.object_name, &grasp_step.object_state);
    grasp_step.ee_trajectory.push_back(hand.contact_pose);
    program_.steps.push_back(grasp_step);
  } else if (hand.current_action == msgs::HandState::GRASPING &&
             prev_hand.current_action == msgs::HandState::GRASPING) {
    msgs::Step* prev_step = GetMostRecentStep(arm_name);
    ROS_ASSERT(prev_step != NULL);
    ROS_ASSERT(prev_step->action_type == msgs::Step::GRASP ||
               prev_step->action_type == msgs::Step::FOLLOW_TRAJECTORY);
    ROS_ASSERT(prev_step->object_state.name == hand.object_name);

    // If previous step was a grasp (meaning this is the first waypoint in the
    // trajectory), insert a trajectory step. Otherwise, use the existing one.
    msgs::Step* traj_step;
    if (prev_step->action_type == msgs::Step::GRASP) {
      msgs::Step inserted_traj_step;
      inserted_traj_step.start_time =
          prev_step->start_time + ros::Duration(kGraspDuration);
      inserted_traj_step.arm = arm_name;
      inserted_traj_step.action_type = msgs::Step::FOLLOW_TRAJECTORY;
      inserted_traj_step.object_state = prev_step->object_state;
      program_.steps.push_back(inserted_traj_step);
      traj_step = &program_.steps[program_.steps.size() - 1];
    } else {
      traj_step = prev_step;
    }

    // Compute pose of the end-effector.
    msgs::ObjectState object_state;
    GetObjectState(state, hand.object_name, &object_state);
    tg::Graph graph;
    graph.Add("initial object pose", tg::RefFrame("camera"),
              traj_step->object_state.pose);
    graph.Add("current object pose", tg::RefFrame("camera"), object_state.pose);
    graph.Add("current grasp", tg::RefFrame("current object pose"),
              hand.contact_pose);
    tg::Transform ee_transform;
    graph.ComputeDescription(tg::LocalFrame("current object pose"),
                             tg::RefFrame("camera"), &ee_transform);
    geometry_msgs::Pose ee_pose;
    ee_transform.ToPose(&ee_pose);
    traj_step->ee_trajectory.push_back(ee_pose);

    // If this is the first waypoint, time from start is just dt.
    // Otherwise, it's last waypoint + dt.
    ros::Duration dt = state.stamp - prev_state_.stamp;
    ros::Duration time_from_step_start;
    if (prev_step->action_type == msgs::Step::GRASP) {
      time_from_step_start = dt;
    } else {
      time_from_step_start = traj_step->times_from_start.back() + dt;
    }
    traj_step->times_from_start.push_back(time_from_step_start);
  } else if (hand.current_action == msgs::HandState::NONE &&
             prev_hand.current_action == msgs::HandState::GRASPING) {
    msgs::Step* prev_step = GetMostRecentStep(arm_name);
    ROS_ASSERT(prev_step != NULL);
    ROS_ASSERT(prev_step->action_type == msgs::Step::GRASP ||
               prev_step->action_type == msgs::Step::FOLLOW_TRAJECTORY);

    ros::Duration dt = state.stamp - prev_state_.stamp;
    msgs::Step ungrasp_step;
    ungrasp_step.start_time = GetEndTime(*prev_step) + dt;
    ungrasp_step.arm = arm_name;
    ungrasp_step.action_type = msgs::Step::UNGRASP;
    program_.steps.push_back(ungrasp_step);
  }
}

msgs::Step* ProgramGenerator::GetMostRecentStep(const std::string& arm_name) {
  for (int i = program_.steps.size() - 1; i >= 0; --i) {
    const msgs::Step& step = program_.steps[i];
    if (step.arm == arm_name) {
      return &program_.steps[i];
    }
  }
  return NULL;
}

void ProgramGenerator::GetObjectState(const msgs::DemoState& state,
                                      const std::string& object_name,
                                      msgs::ObjectState* object_state) {
  for (size_t i = 0; i < state.object_states.size(); ++i) {
    const msgs::ObjectState object = state.object_states[i];
    if (object.name == object_name) {
      *object_state = object;
      return;
    }
  }
  ROS_ERROR("No object \"%s\" in state!", object_name.c_str());
}

ros::Duration ProgramGenerator::GetEndTime(
    const task_perception_msgs::Step& step) {
  if (step.action_type == msgs::Step::GRASP) {
    return step.start_time + ros::Duration(kGraspDuration);
  } else if (step.action_type == msgs::Step::UNGRASP) {
    return step.start_time + ros::Duration(kUngraspDuration);
  } else if (step.action_type == msgs::Step::FOLLOW_TRAJECTORY) {
    if (step.times_from_start.size() > 0) {
      return step.start_time + step.times_from_start.back();
    } else {
      return step.start_time;
    }
  }
  ROS_ASSERT_MSG(false, "Unknown step type %s", step.action_type.c_str());
  return ros::Duration(0);
}
}  // namespace pbi
