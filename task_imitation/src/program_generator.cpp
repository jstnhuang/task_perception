// Given a demonstration as a sequence of DemoStates, generates a program to
// imitate the demonstration.
#include "task_imitation/program_generator.h"

#include "boost/foreach.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "rapid_collision/collision_checks.h"
#include "rapid_ros/params.h"
#include "rapid_utils/vector3.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception/pcl_typedefs.h"
#include "task_perception_msgs/HandState.h"
#include "task_perception_msgs/Step.h"
#include "transform_graph/graph.h"

#include "task_imitation/grasp_planner.h"
#include "task_imitation/grasp_planning_context.h"
#include "task_imitation/program_constants.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using geometry_msgs::Pose;

namespace pbi {

CollisionChecker::CollisionChecker(const std::string& planning_frame)
    : planning_frame_(planning_frame), model_cache_() {}

std::string CollisionChecker::Check(
    const task_perception_msgs::ObjectState& object,
    const std::vector<task_perception_msgs::ObjectState>& other_objects) const {
  const double kInflationSize =
      rapid::GetDoubleParamOrThrow("task_imitation/object_inflation_size");
  LazyObjectModel held_obj_model(object.mesh_name, planning_frame_,
                                 object.pose);
  held_obj_model.set_object_model_cache(&model_cache_);
  const Pose& held_obj_pose = held_obj_model.pose();
  Eigen::Vector3d held_obj_vec = rapid::AsVector3d(held_obj_pose.position);
  geometry_msgs::Vector3 held_obj_scale =
      InflateScale(held_obj_model.scale(), kInflationSize);
  double closest_sq_distance = std::numeric_limits<double>::max();
  std::string closest_collidee("");
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
      double sq_distance = (held_obj_vec - other_vec).squaredNorm();
      if (sq_distance < closest_sq_distance) {
        closest_sq_distance = sq_distance;
        closest_collidee = other.name;
      }
      // ROS_INFO("%s is colliding with %s, dist=%f", object.name.c_str(),
      //         other.name.c_str(), (held_obj_vec - other_vec).norm());
    }
  }
  return closest_collidee;
}

bool CollisionChecker::Check(const msgs::ObjectState& obj1,
                             const msgs::ObjectState& obj2) const {
  LazyObjectModel obj1_model(obj1.mesh_name, planning_frame_, obj1.pose);
  LazyObjectModel obj2_model(obj2.mesh_name, planning_frame_, obj2.pose);
  obj1_model.set_object_model_cache(&model_cache_);
  obj2_model.set_object_model_cache(&model_cache_);
  const double kInflationSize =
      rapid::GetDoubleParamOrThrow("task_imitation/object_inflation_size");

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
    } else {
      ROS_ASSERT(false);
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
    segment.target_object = hand.object_name;
    segment.demo_states.push_back(demo_state);
    segments_->push_back(segment);

    const msgs::ObjectState object =
        GetObjectState(demo_state, hand.object_name);
    std::string collidee(
        collision_checker_.Check(object, demo_state.object_states));
    if (collidee == "") {
      working_move_ = NewMoveToSegment();
      working_move_.demo_states.push_back(demo_state);
      ROS_INFO("%s transitioning from NONE to FREE_GRASP", arm_name_.c_str());
      state_ = FREE_GRASP;
    } else if (other_hand.current_action == msgs::HandState::GRASPING &&
               collidee == other_hand.object_name) {
      working_traj_ = NewTrajSegment();
      working_traj_.target_object =
          InferDoubleCollisionTarget(demo_states_, index_, collision_checker_);
      ROS_INFO("%s transitioning from NONE to DOUBLE_COLLISION",
               arm_name_.c_str());
      state_ = DOUBLE_COLLISION;
    } else {
      working_traj_ = NewTrajSegment();
      working_traj_.target_object = collidee;
      ROS_INFO("%s transitioning from NONE to STATIONARY_COLLISION",
               arm_name_.c_str());
      state_ = STATIONARY_COLLISION;
    }
  }
}

void HandStateMachine::FreeGraspState(const msgs::DemoState& demo_state) {
  msgs::HandState hand = GetHand(demo_state);
  msgs::HandState other_hand = GetOtherHand(demo_state);
  // Handle ungrasp
  if (hand.current_action == msgs::HandState::NONE) {
    ROS_ASSERT(index_ >= 1);
    const msgs::DemoState& prev_state = demo_states_[index_ - 1];
    working_move_.demo_states.push_back(prev_state);
    msgs::HandState prev_hand = GetHand(prev_state);
    working_move_.target_object = prev_hand.object_name;
    segments_->push_back(working_move_);
    working_move_ = NewMoveToSegment();

    ProgramSegment ungrasp_segment = NewUngraspSegment();
    ungrasp_segment.demo_states.push_back(demo_state);
    segments_->push_back(ungrasp_segment);

    ROS_INFO("%s transitioning from FREE_GRASP to NONE", arm_name_.c_str());
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
      working_move_.demo_states.push_back(demo_state);
      working_move_.target_object = collidee;
      segments_->push_back(working_move_);
      working_move_ = NewMoveToSegment();

      if (other_hand.current_action == msgs::HandState::GRASPING &&
          collidee == other_hand.object_name) {
        working_traj_ = NewTrajSegment();
        working_traj_.target_object = InferDoubleCollisionTarget(
            demo_states_, index_, collision_checker_);
        if (working_traj_.target_object != hand.object_name) {
          working_traj_.demo_states.push_back(demo_state);
        }
        ROS_INFO("%s transitioning from FREE_GRASP to DOUBLE_COLLISION",
                 arm_name_.c_str());
        state_ = DOUBLE_COLLISION;
      } else {
        working_traj_ = NewTrajSegment();
        working_traj_.target_object = collidee;
        working_traj_.demo_states.push_back(demo_state);
        ROS_INFO("%s transitioning from FREE_GRASP to STATIONARY_COLLISION",
                 arm_name_.c_str());
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
    ungrasp_segment.demo_states.push_back(demo_state);
    segments_->push_back(ungrasp_segment);
    ROS_INFO("%s transitioning from STATIONARY_COLLISION to NONE",
             arm_name_.c_str());
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
    working_move_ = NewMoveToSegment();
    working_move_.demo_states.push_back(demo_state);
    ROS_INFO("%s transitioning from STATIONARY_COLLISION to FREE_GRASP",
             arm_name_.c_str());
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

    ROS_INFO("%s transitioning from STATIONARY_COLLISION to DOUBLE_COLLISION",
             arm_name_.c_str());
    state_ = DOUBLE_COLLISION;
  }
}

void HandStateMachine::DoubleCollisionState(const msgs::DemoState& demo_state) {
  // Either:
  // 1. and 2. One gripper ungrasps (e.g., stabilized stacking)
  // 3. We are no longer in double collision (e.g., stabilized unstacking).
  // 4. We stay in double collision
  msgs::HandState hand = GetHand(demo_state);
  msgs::HandState other_hand = GetOtherHand(demo_state);
  if (hand.current_action == msgs::HandState::NONE) {
    // If this hand is the master (i.e., not the target), then save the
    // trajectory relative to the target.
    bool is_master = hand.object_name != working_traj_.target_object;
    if (is_master) {
      if (working_traj_.demo_states.size() > 0) {
        segments_->push_back(working_traj_);
      }
      working_traj_ = NewTrajSegment();
      ProgramSegment ungrasp_segment = NewUngraspSegment();
      ungrasp_segment.demo_states.push_back(demo_state);
      segments_->push_back(ungrasp_segment);
    }
    // Regardless of whether this hand is the master or the target, set state to
    // NONE since we ungrasped.
    ROS_INFO("%s transitioning from DOUBLE_COLLISION to NONE",
             arm_name_.c_str());
    state_ = NONE;
  } else if (other_hand.current_action == msgs::HandState::NONE) {
    // If this hand is the master, then continue the trajectory relative to the
    // other object (but transition to STATIONARY COLLISION after). If this hand
    // is the target, then just do nothing.
    bool is_master = hand.object_name != working_traj_.target_object;
    if (is_master) {
      working_traj_.demo_states.push_back(demo_state);
    } else {
      working_traj_ = NewTrajSegment();
    }
    // If the other hand ungrasps, then transition to STATIONARY COLLISION
    ROS_INFO("%s transitioning from DOUBLE_COLLISION to STATIONARY_COLLISION",
             arm_name_.c_str());
    state_ = STATIONARY_COLLISION;
  } else {
    // Both hands are still grasping, check if they are still in collision or
    // not.
    const msgs::ObjectState object =
        GetObjectState(demo_state, hand.object_name);
    std::string collidee(
        collision_checker_.Check(object, demo_state.object_states));

    if (collidee == "") {
      // Free grasping
      segments_->push_back(working_traj_);
      working_traj_ = NewTrajSegment();
      working_move_ = NewMoveToSegment();
      working_move_.demo_states.push_back(demo_state);
      ROS_INFO("%s transitioning from DOUBLE_COLLISION to FREE_GRASP",
               arm_name_.c_str());
      state_ = FREE_GRASP;
    } else if (collidee == other_hand.object_name) {
      // Stay in double collision
      // Update trajectory of master object.
      ROS_ASSERT(other_hand.object_name == working_traj_.target_object);
      bool is_master = hand.object_name != working_traj_.target_object;
      if (is_master) {
        working_traj_.demo_states.push_back(demo_state);
      }
    } else {
      // Stationary collision
      segments_->push_back(working_traj_);
      working_traj_ = NewTrajSegment();
      working_traj_.target_object = collidee;
      working_traj_.demo_states.push_back(demo_state);
      ROS_INFO("%s transitioning from DOUBLE_COLLISION to STATIONARY_COLLISION",
               arm_name_.c_str());
      state_ = STATIONARY_COLLISION;
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

msgs::HandState HandStateMachine::GetPrevHand() {
  if (index_ <= 0) {
    ROS_ERROR("Called GetPrevHand while index=0!");
    ROS_ASSERT(false);
    msgs::HandState blank;
    return blank;
  }
  const msgs::DemoState& demo_state = demo_states_[index_ - 1];
  if (arm_name_ == msgs::Step::LEFT) {
    return demo_state.left_hand;
  } else if (arm_name_ == msgs::Step::RIGHT) {
    return demo_state.right_hand;
  }
  ROS_ASSERT(false);
  msgs::HandState blank;
  return blank;
}

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
      start_time_(0),
      left_group_(left_group),
      right_group_(right_group),
      planning_frame_(left_group_.getPlanningFrame()),
      collision_checker_(planning_frame_) {}

msgs::Program ProgramGenerator::Generate(
    const std::vector<task_perception_msgs::DemoState>& demo_states,
    const ObjectStateIndex& initial_runtime_objects) {
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

  for (size_t i = 0; i < segments.size(); ++i) {
    ProcessSegment(segments[i], initial_runtime_objects, initial_demo_objects);
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
    const ObjectStateIndex& initial_demo_objects) {
  if (segment.type == msgs::Step::GRASP) {
    AddGraspStep(segment, initial_runtime_objects);
  } else if (segment.type == msgs::Step::UNGRASP) {
    AddUngraspStep(segment);
  } else if (segment.type == msgs::Step::MOVE_TO_POSE) {
    AddMoveToStep(segment, initial_demo_objects);
  } else if (segment.type == msgs::Step::FOLLOW_TRAJECTORY) {
    AddTrajectoryStep(segment, initial_runtime_objects);
  } else {
    ROS_ASSERT(false);
  }
}

void ProgramGenerator::AddGraspStep(
    const ProgramSegment& segment,
    const ObjectStateIndex& initial_runtime_objects) {
  const msgs::DemoState& state = segment.demo_states[0];

  msgs::HandState hand;
  if (segment.arm_name == msgs::Step::LEFT) {
    hand = state.left_hand;
  } else if (segment.arm_name == msgs::Step::RIGHT) {
    hand = state.right_hand;
  }

  msgs::Step grasp_step;

  // Compute start time
  int prev_step_index = GetMostRecentStep(segment.arm_name);
  if (prev_step_index != -1) {
    ros::Duration prev_end = GetEndTime(program_.steps[prev_step_index]);
    if (prev_end > grasp_step.start_time) {
      grasp_step.start_time = prev_end + ros::Duration(0.03);
    }
  } else {
    grasp_step.start_time = state.stamp - start_time_;
  }

  grasp_step.arm = segment.arm_name;
  grasp_step.type = msgs::Step::GRASP;
  grasp_step.object_state = GetObjectState(state, hand.object_name);
  const std::string object_name = grasp_step.object_state.name;

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
  GraspPlanner grasp_planner;
  Pose grasp_in_planning = grasp_planner.Plan(context);
  graph.Add("grasp", tg::RefFrame("planning"), grasp_in_planning);
  tg::Transform grasp_in_obj;
  graph.ComputeDescription("grasp", tg::RefFrame("object"), &grasp_in_obj);

  grasp_step.ee_trajectory.push_back(grasp_in_obj.pose());
  program_.steps.push_back(grasp_step);
}

void ProgramGenerator::AddUngraspStep(const ProgramSegment& segment) {
  int prev_step_i = GetMostRecentStep(segment.arm_name);
  ROS_ASSERT(prev_step_i != -1);
  const msgs::Step& prev_step = program_.steps[prev_step_i];
  ROS_ASSERT(prev_step.type == msgs::Step::GRASP ||
             prev_step.type == msgs::Step::FOLLOW_TRAJECTORY ||
             prev_step.type == msgs::Step::MOVE_TO_POSE);

  msgs::Step ungrasp_step;
  ungrasp_step.start_time = GetEndTime(prev_step) + ros::Duration(0.03);
  ungrasp_step.arm = segment.arm_name;
  ungrasp_step.type = msgs::Step::UNGRASP;
  program_.steps.push_back(ungrasp_step);
}

void ProgramGenerator::AddMoveToStep(
    const ProgramSegment& segment,
    const ObjectStateIndex& initial_demo_objects) {
  // Get grasp pose
  int prev_grasp_i = GetMostRecentGraspStep(segment.arm_name);
  ROS_ASSERT(prev_grasp_i != -1);
  const msgs::Step prev_grasp = program_.steps[prev_grasp_i];
  tg::Graph graph;
  graph.Add("gripper", tg::RefFrame("grasped object"),
            prev_grasp.ee_trajectory[0]);

  // Add transform of grasped object relative to target object
  ROS_ASSERT(segment.demo_states.size() == 2);
  const ros::Time start_time(segment.demo_states[0].stamp);
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

  int prev_step_i = GetMostRecentStep(segment.arm_name);
  ROS_ASSERT(prev_step_i != -1);
  const msgs::Step prev_step = program_.steps[prev_step_i];

  msgs::Step move_step;
  move_step.start_time = GetEndTime(prev_step) + ros::Duration(0.03);
  move_step.arm = segment.arm_name;
  move_step.type = msgs::Step::MOVE_TO_POSE;
  move_step.object_state = target_obj;
  move_step.ee_trajectory.push_back(ee_in_target.pose());
  move_step.times_from_start.push_back(end_state.stamp - start_time);
  program_.steps.push_back(move_step);
}

void ProgramGenerator::AddTrajectoryStep(
    const ProgramSegment& segment,
    const ObjectStateIndex& initial_runtime_objects) {
  int prev_step_i = GetMostRecentStep(segment.arm_name);
  ROS_ASSERT(prev_step_i != -1);
  const msgs::Step prev_step = program_.steps[prev_step_i];

  // Set up all fields of trajectory step other than the trajectory itself.
  msgs::Step traj_step;
  traj_step.start_time = GetEndTime(prev_step) + ros::Duration(0.03);
  traj_step.arm = segment.arm_name;
  traj_step.type = msgs::Step::FOLLOW_TRAJECTORY;
  traj_step.object_state = initial_runtime_objects.at(segment.target_object);

  // Get grasp pose
  int prev_grasp_i = GetMostRecentGraspStep(segment.arm_name);
  ROS_ASSERT(prev_grasp_i != -1);
  const msgs::Step prev_grasp = program_.steps[prev_grasp_i];

  tg::Graph graph;
  graph.Add("gripper", tg::RefFrame("grasped object"),
            prev_grasp.ee_trajectory[0]);

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

int ProgramGenerator::GetMostRecentStep(const std::string& arm_name) {
  for (int i = program_.steps.size() - 1; i >= 0; --i) {
    const msgs::Step& step = program_.steps[i];
    if (step.arm == arm_name) {
      return i;
    }
  }
  return -1;
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

ros::Duration ProgramGenerator::GetEndTime(
    const task_perception_msgs::Step& step) {
  if (step.type == msgs::Step::GRASP) {
    return step.start_time + ros::Duration(kGraspDuration);
  } else if (step.type == msgs::Step::UNGRASP) {
    return step.start_time + ros::Duration(kUngraspDuration);
  } else if (step.type == msgs::Step::MOVE_TO_POSE) {
    if (step.times_from_start.size() > 0) {
      return step.start_time + step.times_from_start.back();
    } else {
      ROS_ASSERT(false);
      return step.start_time;
    }
  } else if (step.type == msgs::Step::FOLLOW_TRAJECTORY) {
    if (step.times_from_start.size() > 0) {
      return step.start_time + step.times_from_start.back();
    } else {
      ROS_ASSERT(false);
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
