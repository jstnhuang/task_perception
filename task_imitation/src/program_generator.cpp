// Given a demonstration as a sequence of DemoStates, generates a program to
// imitate the demonstration.
#include "task_imitation/program_generator.h"

#include "boost/foreach.hpp"
#include "boost/optional.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_collision/collision_checks.h"
#include "rapid_ros/params.h"
#include "rapid_utils/vector3_traits.hpp"
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
using boost::optional;
using geometry_msgs::Pose;

namespace pbi {

CollisionChecker::CollisionChecker(const std::string& planning_frame)
    : planning_frame_(planning_frame), model_cache_() {}

std::string CollisionChecker::Check(
    const task_perception_msgs::ObjectState& object,
    const std::vector<task_perception_msgs::ObjectState>& other_objects) const {
  const double kInflationSize =
      rapid::GetDoubleParamOrThrow("object_inflation_size");
  LazyObjectModel held_obj_model(object.mesh_name, planning_frame_,
                                 object.pose);
  held_obj_model.set_object_model_cache(&model_cache_);
  const geometry_msgs::Pose& held_obj_pose = held_obj_model.pose();
  Eigen::Vector3d held_obj_vec = rapid::AsVector3d(held_obj_pose.position);
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
      Eigen::Vector3d other_vec =
          rapid::AsVector3d(other_model.pose().position);
      ROS_INFO("%s is colliding with %s, dist=%f", object.name.c_str(),
               other.name.c_str(), (held_obj_vec - other_vec).norm());
      return other.name;
    }
  }
  return "";
}

bool CollisionChecker::Check(const msgs::ObjectState& obj1,
                             const msgs::ObjectState& obj2) const {
  LazyObjectModel obj1_model(obj1.mesh_name, planning_frame_, obj1.pose);
  LazyObjectModel obj2_model(obj2.mesh_name, planning_frame_, obj2.pose);
  const double kInflationSize =
      rapid::GetDoubleParamOrThrow("object_inflation_size");

  geometry_msgs::Vector3 obj1_scale =
      InflateScale(obj1_model.scale(), kInflationSize);
  geometry_msgs::Vector3 obj2_scale =
      InflateScale(obj2_model.scale(), kInflationSize);
  return rapid::AreObbsInCollision(obj1_model.pose(), obj1_scale,
                                   obj2_model.pose(), obj2_scale);
}

HandStateMachine::HandStateMachine(
    const std::vector<msgs::DemoState>& demo_states,
    const std::string& arm_name, const CollisionChecker& collision_checker,
    std::vector<ProgramSegment>* segments)
    : demo_states_(demo_states),
      arm_name_(arm_name),
      collision_checker_(collision_checker),
      segments_(segments),
      state_(NONE),
      index_(0),
      working_traj_() {}

bool HandStateMachine::Step() {
  if (static_cast<size_t>(index_) < demo_states_.size()) {
    const msgs::DemoState& demo_state = demo_states_[index_];
    if (state_ == NONE) {
      NoneState(demo_state);
    } else if (state_ == FREE_GRASP) {
      FreeGraspState(demo_state);
    } else if (state_ == STATIONARY_COLLISION) {
      StationaryCollisionState(demo_state);
    } else if (state_ == DOUBLE_COLLISION) {
      DoubleCollisionState(demo_state);
    }
    ++index_;
    return true;
  }
  return false;
}

void HandStateMachine::NoneState(const msgs::DemoState& demo_state) {
  msgs::HandState hand = GetHand(demo_state);
  msgs::HandState other_hand = GetOtherHand(demo_state);

  if (hand.current_action == msgs::HandState::GRASPING) {
    ProgramSegment segment = NewGraspSegment();
    segment.demo_states.push_back(demo_state);

    const msgs::ObjectState object =
        GetObjectState(demo_state, hand.object_name);
    std::string collidee(
        collision_checker_.Check(object, demo_state.object_states));
    if (collidee == "") {
      state_ = FREE_GRASP;
    } else if (other_hand.current_action == msgs::HandState::GRASPING ||
               collidee != other_hand.object_name) {
      state_ = STATIONARY_COLLISION;
    } else if (other_hand.current_action == msgs::HandState::GRASPING &&
               collidee == other_hand.object_name) {
      state_ = DOUBLE_COLLISION;
    }
  }
}

void HandStateMachine::FreeGraspState(const msgs::DemoState& demo_state) {
  msgs::HandState hand = GetHand(demo_state);
  msgs::HandState other_hand = GetOtherHand(demo_state);
  // Handle ungrasp
  if (hand.current_action == msgs::HandState::NONE) {
    ProgramSegment move_segment = NewMoveToSegment();
    move_segment.demo_states.push_back(demo_state);
    // TODO: handle initial vs. current object poses
    move_segment.target_object = "initial " + hand.object_name;

    ProgramSegment ungrasp_segment = NewUngraspSegment();
    segments_->push_back(move_segment);
    segments_->push_back(ungrasp_segment);

    state_ = NONE;
  }

  // Handle collision
  // If we collide with an object, this means the hand was moving through free
  // space but is now close to another object. We add a segment that directly
  // moves the arm to the pose where they intersected.
  else {
    const msgs::ObjectState object =
        GetObjectState(demo_state, hand.object_name);
    std::string collidee(
        collision_checker_.Check(object, demo_state.object_states));
    if (collidee != "") {
      ProgramSegment move_segment = NewMoveToSegment();
      move_segment.demo_states.push_back(demo_state);
      move_segment.target_object = "current " + collidee;
      segments_->push_back(move_segment);

      if (other_hand.current_action == msgs::HandState::GRASPING &&
          collidee == other_hand.object_name) {
        working_traj_ = NewTrajSegment();
        working_traj_.target_object = InferDoubleCollisionTarget(
            demo_states_, index_, collision_checker_);
        if (working_traj_.target_object != hand.object_name) {
          working_traj_.demo_states.push_back(demo_state);
        }
        state_ = DOUBLE_COLLISION;
      } else {
        working_traj_ = NewTrajSegment();
        working_traj_.target_object = collidee;
        working_traj_.demo_states.push_back(demo_state);
        state_ = STATIONARY_COLLISION;
      }
    }
  }
}

void HandStateMachine::StationaryCollisionState(
    const msgs::DemoState& demo_state) {
  // Ungrasp -> None
  // Exit collision -> Free grasping
  // Enter collision with different object -> Stationary collision, but start
  // new trajectory
  // Enter collision with held object
  msgs::HandState hand = GetHand(demo_state);
  msgs::HandState other_hand = GetOtherHand(demo_state);

  // Handle ungrasp
  if (hand.current_action == msgs::HandState::NONE) {
    segments_->push_back(working_traj_);
    working_traj_ = NewTrajSegment();

    ProgramSegment ungrasp_segment = NewUngraspSegment();
    segments_->push_back(ungrasp_segment);
    state_ = NONE;
    return;
  }

  // Check collisions. Either:
  // 1. We exit collision
  // 2. We stay in collision with the same object
  // 3. We enter collision with a new stationary object
  // 4. We enter double collision (the other object is held in the other hand)
  const msgs::ObjectState object = GetObjectState(demo_state, hand.object_name);
  std::string collidee(
      collision_checker_.Check(object, demo_state.object_states));
  // Exiting collision
  if (collidee == "") {
    segments_->push_back(working_traj_);
    working_traj_ = NewTrajSegment();
    state_ = FREE_GRASP;
  }
  // Staying in collision with the same object
  else if (collidee == working_traj_.target_object) {
    working_traj_.demo_states.push_back(demo_state);
  }
  // Enter collision with a different stationary object
  else if (collidee != working_traj_.target_object &&
           collidee != other_hand.object_name) {
    segments_->push_back(working_traj_);
    working_traj_ = NewTrajSegment();
    working_traj_.target_object = collidee;
    working_traj_.demo_states.push_back(demo_state);
  }
  // Enter collision with object held by other hand
  else if (collidee != working_traj_.target_object &&
           other_hand.current_action == msgs::HandState::GRASPING &&
           collidee == other_hand.object_name) {
    segments_->push_back(working_traj_);
    working_traj_ = NewTrajSegment();
    working_traj_.target_object =
        InferDoubleCollisionTarget(demo_states_, index_, collision_checker_);
    if (working_traj_.target_object != hand.object_name) {
      working_traj_.demo_states.push_back(demo_state);
    }

    state_ = DOUBLE_COLLISION;
  }
}

void HandStateMachine::DoubleCollisionState(const msgs::DemoState& demo_state) {
  // Either: One gripper ungrasps (e.g., stabilized stacking) or we are no
  // longer in double collision (e.g., stabilized unstacking).
  msgs::HandState hand = GetHand(demo_state);
  msgs::HandState other_hand = GetOtherHand(demo_state);
  if (hand.current_action == msgs::HandState::NONE) {
    // If this hand is the master (i.e., not the target), then save the
    // trajectory relative to the target.
    if (hand.object_name != working_traj_.target_object) {
      if (working_traj_.demo_states.size() > 0) {
        segments_->push_back(working_traj_);
      }
    }
    working_traj_ = NewTrajSegment();
    ProgramSegment ungrasp_segment = NewUngraspSegment();
    segments_->push_back(ungrasp_segment);
    state_ = NONE;
  } else if (other_hand.current_action == msgs::HandState::NONE) {
    // If this hand is the master, then continue the trajectory relative to the
    // other object (but transition to STATIONARY COLLISION after). The hand
    // holding the target object just throws away its trajectory.
    if (hand.object_name != working_traj_.target_object) {
      working_traj_.demo_states.push_back(demo_state);
    } else {
      working_traj_ = NewTrajSegment();
    }
    // If the other hand ungrasps, then transition to STATIONARY COLLISION
    state_ = STATIONARY_COLLISION;
  } else {
    // Update trajectory of master object.
    if (hand.object_name != working_traj_.target_object) {
      working_traj_.demo_states.push_back(demo_state);
    }
  }
}

msgs::HandState HandStateMachine::GetHand(const msgs::DemoState& demo_state) {
  if (arm_name_ == msgs::Step::LEFT) {
    return demo_state.left_hand;
  } else if (arm_name_ == msgs::Step::RIGHT) {
    return demo_state.right_hand;
  }
  ROS_ASSERT(false);
  msgs::HandState blank;
  return blank;
}

msgs::HandState HandStateMachine::GetOtherHand(
    const msgs::DemoState& demo_state) {
  if (arm_name_ == msgs::Step::LEFT) {
    return demo_state.right_hand;
  } else if (arm_name_ == msgs::Step::RIGHT) {
    return demo_state.left_hand;
  }
  ROS_ASSERT(false);
  msgs::HandState blank;
  return blank;
}

const double ProgramGenerator::kGraspDuration = 2;
const double ProgramGenerator::kUngraspDuration = 2;

ProgramSegment HandStateMachine::NewGraspSegment() {
  ProgramSegment segment;
  segment.arm_name = arm_name_;
  segment.type = msgs::Step::GRASP;
  return segment;
}

ProgramSegment HandStateMachine::NewUngraspSegment() {
  ProgramSegment segment;
  segment.arm_name = arm_name_;
  segment.type = msgs::Step::UNGRASP;
  return segment;
}

ProgramSegment HandStateMachine::NewMoveToSegment() {
  ProgramSegment segment;
  segment.arm_name = arm_name_;
  segment.type = msgs::Step::MOVE_TO_POSE;
  return segment;
}

ProgramSegment HandStateMachine::NewTrajSegment() {
  ProgramSegment segment;
  segment.arm_name = arm_name_;
  segment.type = msgs::Step::FOLLOW_TRAJECTORY;
  return segment;
}

ProgramGenerator::ProgramGenerator(
    moveit::planning_interface::MoveGroup& left_group,
    moveit::planning_interface::MoveGroup& right_group)
    : program_(),
      prev_state_(),
      start_time_(0),
      left_group_(left_group),
      right_group_(right_group),
      planning_frame_(left_group_.getPlanningFrame()),
      collision_checker_(planning_frame_) {}

msgs::Program ProgramGenerator::Generate(
    const std::vector<task_perception_msgs::DemoState>& demo_states,
    const ObjectStateIndex& initial_objects) {
  std::vector<ProgramSegment> segments = Segment(demo_states, initial_objects);
  for (size_t i = 0; i < demo_states.size(); ++i) {
    Step(demo_states[i], initial_objects);
  }
  return program_;
}

std::vector<ProgramSegment> ProgramGenerator::Segment(
    const std::vector<msgs::DemoState>& demo_states,
    const ObjectStateIndex& initial_objects) {
  std::vector<ProgramSegment> segments;
  HandStateMachine left(demo_states, msgs::Step::LEFT, collision_checker_,
                        &segments);
  HandStateMachine right(demo_states, msgs::Step::LEFT, collision_checker_,
                         &segments);
  bool continue_left = false;
  bool continue_right = false;
  do {
    continue_left = left.Step();
    continue_right = right.Step();
    ROS_ASSERT(continue_left == continue_right);
  } while (continue_left && continue_right);
  return segments;
}

boost::optional<ProgramSegment> ProgramGenerator::SegmentGrasp(
    const task_perception_msgs::DemoState& state,
    const task_perception_msgs::DemoState& prev_state,
    const std::string& arm_name) {
  msgs::HandState hand;
  msgs::HandState prev_hand;
  if (arm_name == msgs::Step::LEFT) {
    hand = state.left_hand;
    prev_hand = prev_state.left_hand;
  } else if (arm_name == msgs::Step::RIGHT) {
    hand = state.right_hand;
    prev_hand = prev_state.right_hand;
  }
  if (hand.current_action == msgs::HandState::GRASPING &&
      prev_hand.current_action == msgs::HandState::NONE) {
    ProgramSegment segment;
    segment.arm_name = arm_name;
    segment.type = msgs::Step::GRASP;
    segment.demo_states.push_back(state);
    return segment;
  } else {
    return boost::none;
  }
}

boost::optional<ProgramSegment> ProgramGenerator::SegmentUngrasp(
    const task_perception_msgs::DemoState& state,
    const task_perception_msgs::DemoState& prev_state,
    const std::string& arm_name) {
  msgs::HandState hand;
  msgs::HandState prev_hand;
  if (arm_name == msgs::Step::LEFT) {
    hand = state.left_hand;
    prev_hand = prev_state.left_hand;
  } else if (arm_name == msgs::Step::RIGHT) {
    hand = state.right_hand;
    prev_hand = prev_state.right_hand;
  }
  if (hand.current_action == msgs::HandState::NONE &&
      prev_hand.current_action == msgs::HandState::GRASPING) {
    ProgramSegment segment;
    segment.arm_name = arm_name;
    segment.type = msgs::Step::UNGRASP;
    segment.demo_states.push_back(state);
    return segment;
  } else {
    return boost::none;
  }
}

void ProgramGenerator::Step(const msgs::DemoState& state,
                            const ObjectStateIndex& initial_objects) {
  ProcessStep(state, initial_objects, msgs::Step::LEFT);
  ProcessStep(state, initial_objects, msgs::Step::RIGHT);
  prev_state_ = state;
}

void ProgramGenerator::ProcessStep(const msgs::DemoState& state,
                                   const ObjectStateIndex& initial_objects,
                                   const std::string& arm_name) {
  msgs::HandState hand;
  msgs::HandState other_hand;
  msgs::HandState prev_hand;
  if (arm_name == msgs::Step::LEFT) {
    hand = state.left_hand;
    other_hand = state.right_hand;
    prev_hand = prev_state_.left_hand;
  } else if (arm_name == msgs::Step::RIGHT) {
    hand = state.right_hand;
    other_hand = state.left_hand;
    prev_hand = prev_state_.right_hand;
  }

  // If we start a contact, then add a grasp step to the program
  if (hand.current_action == msgs::HandState::GRASPING &&
      (prev_hand.current_action == "" ||
       prev_hand.current_action == msgs::HandState::NONE)) {
    AddGraspStep(state, initial_objects, arm_name);
  }

  // If we are continuing a contact, then create/append to the trajectory
  else if (hand.current_action == msgs::HandState::GRASPING &&
           prev_hand.current_action == msgs::HandState::GRASPING) {
    AddOrAppendToTrajectoryStep(state, initial_objects, arm_name);
  }

  // If we are ending a contact, then end the trajectory
  else if (hand.current_action == msgs::HandState::NONE &&
           prev_hand.current_action == msgs::HandState::GRASPING) {
    AddUngraspStep(state, initial_objects, arm_name);
  }
}

void ProgramGenerator::AddGraspStep(const msgs::DemoState& state,
                                    const ObjectStateIndex& initial_objects,
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
  const Pose& current_obj_pose = initial_objects.at(object_name).pose;
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
    const msgs::DemoState& state, const ObjectStateIndex& initial_objects,
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
                                      const ObjectStateIndex& initial_objects,
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

std::string InferDoubleCollisionTarget(
    const std::vector<task_perception_msgs::DemoState>& demo_states,
    int start_index, const CollisionChecker& collision_checker) {
  // While the two hands are in collision, compute path length
  double left_length = 0;
  double right_length = 0;

  const msgs::DemoState& prev_state = demo_states[start_index];
  const msgs::HandState& prev_left = prev_state.left_hand;
  const msgs::HandState& prev_right = prev_state.right_hand;
  msgs::ObjectState prev_left_obj =
      GetObjectState(prev_state, prev_left.object_name);
  msgs::ObjectState prev_right_obj =
      GetObjectState(prev_state, prev_right.object_name);

  for (size_t i = start_index + 1; i < demo_states.size(); ++i) {
    const msgs::DemoState& demo_state = demo_states[i];
    const msgs::HandState& left = demo_state.left_hand;
    const msgs::HandState& right = demo_state.right_hand;
    if (left.current_action != msgs::HandState::GRASPING ||
        right.current_action != msgs::HandState::GRASPING) {
      break;
    }
    const msgs::ObjectState& left_obj =
        GetObjectState(demo_state, left.object_name);
    const msgs::ObjectState& right_obj =
        GetObjectState(demo_state, right.object_name);
    if (!collision_checker.Check(left_obj, right_obj)) {
      break;
    }

    Eigen::Vector3d left_vec = rapid::AsVector3d(left_obj.pose.position);
    Eigen::Vector3d right_vec = rapid::AsVector3d(right_obj.pose.position);

    Eigen::Vector3d prev_left_vec =
        rapid::AsVector3d(prev_left_obj.pose.position);
    Eigen::Vector3d prev_right_vec =
        rapid::AsVector3d(prev_right_obj.pose.position);
    left_length += (left_vec - prev_left_vec).norm();
    right_length += (right_vec - prev_right_vec).norm();

    prev_left_obj = left_obj;
    prev_right_obj = right_obj;
  }
  if (left_length > right_length) {
    return prev_left_obj.name;
  } else {
    return prev_right_obj.name;
  }
}
}  // namespace pbi
