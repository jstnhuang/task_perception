// Given a demonstration as a sequence of DemoStates, generates a program to
// imitate the demonstration.
#include "task_imitation/program_generator.h"

#include "boost/foreach.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_collision/collision_checks.h"
#include "ros/ros.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception/pcl_typedefs.h"
#include "task_perception_msgs/HandState.h"
#include "task_perception_msgs/Step.h"
#include "transform_graph/graph.h"

#include "task_imitation/grasp_planner.h"
#include "task_imitation/grasp_planning_context.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using geometry_msgs::Pose;

namespace pbi {
const double ProgramGenerator::kGraspDuration = 2;
const double ProgramGenerator::kUngraspDuration = 2;

ProgramGenerator::ProgramGenerator(
    moveit::planning_interface::MoveGroup& left_group,
    moveit::planning_interface::MoveGroup& right_group)
    : program_(),
      prev_state_(),
      start_time_(0),
      left_group_(left_group),
      right_group_(right_group),
      planning_frame_(left_group_.getPlanningFrame()),
      model_cache_() {}

msgs::Program ProgramGenerator::Generate(
    const std::vector<task_perception_msgs::DemoState>& demo_states,
    const ObjectStateIndex& object_states) {
  Segment(demo_states);
  for (size_t i = 0; i < demo_states.size(); ++i) {
    Step(demo_states[i], object_states);
  }
  return program_;
}

// Segments a demonstration based on when objects are touching or in the sphere
// of influence.
void ProgramGenerator::Segment(
    const std::vector<msgs::DemoState>& demo_states) {
  for (size_t i = 0; i < demo_states.size(); ++i) {
    const msgs::DemoState& demo_state = demo_states[i];
    if (demo_state.left_hand.current_action != msgs::HandState::NONE) {
      msgs::ObjectState held_object =
          GetObjectState(demo_state, demo_state.left_hand.object_name);
      CheckContacts(held_object, demo_state.object_states);
    }
    if (demo_state.right_hand.current_action != msgs::HandState::NONE) {
      msgs::ObjectState held_object =
          GetObjectState(demo_state, demo_state.right_hand.object_name);
      CheckContacts(held_object, demo_state.object_states);
    }
  }
}

void ProgramGenerator::CheckContacts(
    const task_perception_msgs::ObjectState& object,
    const std::vector<task_perception_msgs::ObjectState>& other_objects) {
  const double kInflationSize = 0.05;  // Add 0.025 all around
  LazyObjectModel held_obj_model(object.mesh_name, planning_frame_,
                                 object.pose);
  held_obj_model.set_object_model_cache(&model_cache_);
  const geometry_msgs::Pose& held_obj_pose = held_obj_model.pose();
  geometry_msgs::Vector3 held_obj_scale =
      InflateScale(held_obj_model.scale(), kInflationSize);
  BOOST_FOREACH (const msgs::ObjectState& other, other_objects) {
    if (other.name == object.name) {
      continue;
    }
    LazyObjectModel other_model(other.mesh_name, planning_frame_, other.pose);
    other_model.set_object_model_cache(&model_cache_);
    if (rapid::AreObbsInCollision(
            held_obj_pose, held_obj_scale, other_model.pose(),
            InflateScale(other_model.scale(), kInflationSize))) {
      ROS_INFO("%s is colliding with %s", object.name.c_str(),
               other.name.c_str());
    }
  }
}

void ProgramGenerator::Step(const msgs::DemoState& state,
                            const ObjectStateIndex& object_states) {
  ProcessStep(state, object_states, msgs::Step::LEFT);
  ProcessStep(state, object_states, msgs::Step::RIGHT);
  prev_state_ = state;
}

void ProgramGenerator::ProcessStep(const msgs::DemoState& state,
                                   const ObjectStateIndex& object_states,
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

  // If we start a contact, then add a grasp step to the program
  if (hand.current_action == msgs::HandState::GRASPING &&
      (prev_hand.current_action == "" ||
       prev_hand.current_action == msgs::HandState::NONE)) {
    AddGraspStep(state, object_states, arm_name);
  }

  // If we are continuing a contact, then create/append to the trajectory
  else if (hand.current_action == msgs::HandState::GRASPING &&
           prev_hand.current_action == msgs::HandState::GRASPING) {
    AddOrAppendToTrajectoryStep(state, object_states, arm_name);
  }

  // If we are ending a contact, then end the trajectory
  else if (hand.current_action == msgs::HandState::NONE &&
           prev_hand.current_action == msgs::HandState::GRASPING) {
    AddUngraspStep(state, object_states, arm_name);
  }
}

void ProgramGenerator::AddGraspStep(const msgs::DemoState& state,
                                    const ObjectStateIndex& object_states,
                                    const std::string& arm_name) {
  if (program_.steps.size() == 0) {
    start_time_ = state.stamp;
  }

  msgs::HandState hand;
  msgs::HandState prev_hand;
  if (arm_name == msgs::Step::LEFT) {
    hand = state.left_hand;
    prev_hand = prev_state_.left_hand;
  } else if (arm_name == msgs::Step::RIGHT) {
    hand = state.right_hand;
    prev_hand = prev_state_.right_hand;
  }

  msgs::Step grasp_step;

  // Compute start time
  int prev_step_index = GetMostRecentStep(arm_name);
  grasp_step.start_time = state.stamp - start_time_;
  if (prev_step_index != -1) {
    ros::Duration prev_end = GetEndTime(program_.steps[prev_step_index]);
    if (prev_end > grasp_step.start_time) {
      ros::Duration dt = state.stamp - prev_state_.stamp;
      grasp_step.start_time = prev_end + dt;
    }
  }

  grasp_step.arm = arm_name;
  grasp_step.type = msgs::Step::GRASP;
  grasp_step.object_state = GetObjectState(state, hand.object_name);
  const std::string object_name = grasp_step.object_state.name;

  // Plan grasp
  tg::Graph graph;
  const Pose& current_obj_pose = object_states.at(object_name).pose;
  graph.Add("object", tg::RefFrame("planning"), current_obj_pose);
  tg::Transform wrist_in_planning;
  graph.DescribePose(hand.wrist_pose, tg::Source("object"),
                     tg::Target("planning"), &wrist_in_planning);

  GraspPlanningContext context(wrist_in_planning.pose(), planning_frame_,
                               object_name, grasp_step.object_state.mesh_name,
                               current_obj_pose);
  GraspPlanner grasp_planner;
  Pose grasp_in_planning = grasp_planner.Plan(context);
  graph.Add("grasp", tg::RefFrame("planning"), grasp_in_planning);
  tg::Transform grasp_in_obj;
  graph.ComputeDescription("grasp", tg::RefFrame("object"), &grasp_in_obj);

  grasp_step.ee_trajectory.push_back(grasp_in_obj.pose());
  program_.steps.push_back(grasp_step);
}

void ProgramGenerator::AddOrAppendToTrajectoryStep(
    const msgs::DemoState& state, const ObjectStateIndex& object_states,
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

  int prev_step_i = GetMostRecentStep(arm_name);
  ROS_ASSERT(prev_step_i != -1);
  const msgs::Step& prev_step = program_.steps[prev_step_i];
  ROS_ASSERT(prev_step.type == msgs::Step::GRASP ||
             prev_step.type == msgs::Step::FOLLOW_TRAJECTORY);
  ROS_ASSERT(prev_step.object_state.name == hand.object_name);

  // If previous step was a grasp (meaning this is the first waypoint in the
  // trajectory), insert a trajectory step. Otherwise, use the existing one.
  msgs::Step traj_step;
  int traj_step_i = 0;
  if (prev_step.type == msgs::Step::GRASP) {
    traj_step.start_time = prev_step.start_time + ros::Duration(kGraspDuration);
    traj_step.arm = arm_name;
    traj_step.type = msgs::Step::FOLLOW_TRAJECTORY;
    traj_step.object_state = prev_step.object_state;
    program_.steps.push_back(traj_step);
    traj_step_i = program_.steps.size() - 1;
  } else {
    traj_step = prev_step;
    traj_step_i = prev_step_i;
  }

  // Compute pose of the gripper relative to the object's initial pose.
  // TODO: for objects with symmetry (e.g., cylinders), plan new grasps that
  // ignore possible vision system errors about the axis of symmetry.
  msgs::Step grasp_step = GetMostRecentGraspStep(arm_name);
  const Pose& grasp_pose = grasp_step.ee_trajectory[0];  // Relative to obj
  msgs::ObjectState object_state = GetObjectState(state, hand.object_name);
  tg::Graph graph;
  graph.Add("initial object pose", tg::RefFrame("camera"),
            grasp_step.object_state.pose);
  graph.Add("current object pose", tg::RefFrame("camera"), object_state.pose);
  graph.Add("grasp", tg::RefFrame("current object pose"), grasp_pose);
  tg::Transform grasp_in_initial;
  graph.ComputeDescription("grasp", tg::RefFrame("initial object pose"),
                           &grasp_in_initial);
  traj_step.ee_trajectory.push_back(grasp_in_initial.pose());

  // If this is the first waypoint, time from start is just dt.
  // Otherwise, it's last waypoint + dt.
  ros::Duration dt = state.stamp - prev_state_.stamp;
  ros::Duration time_from_step_start;
  if (traj_step.times_from_start.size() == 0) {
    time_from_step_start = dt;
  } else {
    time_from_step_start = traj_step.times_from_start.back() + dt;
  }
  traj_step.times_from_start.push_back(time_from_step_start);
  program_.steps[traj_step_i] = traj_step;
}

void ProgramGenerator::AddUngraspStep(const msgs::DemoState& state,
                                      const ObjectStateIndex& object_states,
                                      const std::string& arm_name) {
  int prev_step_i = GetMostRecentStep(arm_name);
  ROS_ASSERT(prev_step_i != -1);
  const msgs::Step& prev_step = program_.steps[prev_step_i];
  ROS_ASSERT(prev_step.type == msgs::Step::GRASP ||
             prev_step.type == msgs::Step::FOLLOW_TRAJECTORY);

  ros::Duration dt = state.stamp - prev_state_.stamp;
  msgs::Step ungrasp_step;
  ungrasp_step.start_time = GetEndTime(prev_step) + dt;
  ungrasp_step.arm = arm_name;
  ungrasp_step.type = msgs::Step::UNGRASP;
  program_.steps.push_back(ungrasp_step);
}

int ProgramGenerator::GetMostRecentStep(const std::string& arm_name) {
  for (int i = program_.steps.size() - 1; i >= 0; --i) {
    const msgs::Step& step = program_.steps[i];
    if (step.arm == arm_name) {
      return i;
    }
  }
  return -1;
}

msgs::Step ProgramGenerator::GetMostRecentGraspStep(
    const std::string& arm_name) {
  for (int i = program_.steps.size() - 1; i >= 0; --i) {
    const msgs::Step& step = program_.steps[i];
    if (step.type == msgs::Step::GRASP && step.arm == arm_name) {
      return step;
    }
  }
  msgs::Step kBlank;
  ROS_ERROR("Failed to find most recent grasp pose for %s!", arm_name.c_str());
  return kBlank;
}

ros::Duration ProgramGenerator::GetEndTime(
    const task_perception_msgs::Step& step) {
  if (step.type == msgs::Step::GRASP) {
    return step.start_time + ros::Duration(kGraspDuration);
  } else if (step.type == msgs::Step::UNGRASP) {
    return step.start_time + ros::Duration(kUngraspDuration);
  } else if (step.type == msgs::Step::FOLLOW_TRAJECTORY) {
    if (step.times_from_start.size() > 0) {
      return step.start_time + step.times_from_start.back();
    } else {
      return step.start_time;
    }
  }
  ROS_ASSERT_MSG(false, "Unknown step type %s", step.type.c_str());
  return ros::Duration(0);
}

geometry_msgs::Vector3 InflateScale(const geometry_msgs::Vector3& scale,
                                    double distance) {
  geometry_msgs::Vector3 inflated = scale;
  inflated.x += distance;
  inflated.y += distance;
  inflated.z += distance;
  return inflated;
}

msgs::ObjectState GetObjectState(const msgs::DemoState& state,
                                 const std::string& object_name) {
  for (size_t i = 0; i < state.object_states.size(); ++i) {
    const msgs::ObjectState object = state.object_states[i];
    if (object.name == object_name) {
      return object;
    }
  }
  ROS_ERROR("No object \"%s\" in state!", object_name.c_str());
  msgs::ObjectState kBlank;
  return kBlank;
}
}  // namespace pbi
