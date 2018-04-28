#include "task_imitation/hand_state_machine.h"

#include "Eigen/Dense"
#include "rapid_utils/vector3.hpp"
#include "ros/ros.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception_msgs/Step.h"

#include "task_imitation/demo_state.h"

namespace msgs = task_perception_msgs;

namespace pbi {
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
      working_move_(NewMoveToSegment()),
      working_traj_(NewTrajSegment()) {}

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
      ROS_INFO("%d: %s transitioning from NONE to FREE_GRASP", index_,
               arm_name_.c_str());
      state_ = FREE_GRASP;
    } else if (other_hand.current_action == msgs::HandState::GRASPING &&
               collidee == other_hand.object_name) {
      working_traj_ = NewTrajSegment();
      working_traj_.target_object =
          InferDoubleCollisionTarget(demo_states_, index_, collision_checker_);
      ROS_INFO("%d: %s transitioning from NONE to DOUBLE_COLLISION", index_,
               arm_name_.c_str());
      ROS_INFO("Target object is %s", working_traj_.target_object.c_str());
      state_ = DOUBLE_COLLISION;
    } else {
      working_traj_ = NewTrajSegment();
      working_traj_.target_object = collidee;
      ROS_INFO("%d: %s transitioning from NONE to STATIONARY_COLLISION", index_,
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
    ROS_ASSERT(working_move_.target_object != "");
    segments_->push_back(working_move_);
    working_move_ = NewMoveToSegment();

    ProgramSegment ungrasp_segment = NewUngraspSegment();
    ungrasp_segment.demo_states.push_back(demo_state);
    segments_->push_back(ungrasp_segment);

    ROS_INFO("%d: %s transitioning from FREE_GRASP to NONE", index_,
             arm_name_.c_str());
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
      if (other_hand.current_action == msgs::HandState::GRASPING &&
          collidee == other_hand.object_name) {
        // If double collision, then the hand holding the target does nothing.
        // For the master, save a move-to relative to the target and start a new
        // trajectory relative to the target.
        std::string target = InferDoubleCollisionTarget(demo_states_, index_,
                                                        collision_checker_);
        bool is_master = hand.object_name != target;
        if (is_master) {
          working_move_.demo_states.push_back(demo_state);
          working_move_.target_object = collidee;
          ROS_ASSERT(working_move_.target_object != "");
          segments_->push_back(working_move_);
          working_move_ = NewMoveToSegment();

          working_traj_ = NewTrajSegment();
          working_traj_.target_object = target;
          working_traj_.demo_states.push_back(demo_state);
        }
        ROS_INFO("%d: %s transitioning from FREE_GRASP to DOUBLE_COLLISION",
                 index_, arm_name_.c_str());
        ROS_INFO("Target object is \"%s\"", target.c_str());
        state_ = DOUBLE_COLLISION;
      } else {
        // If stationary collision, then save a move-to segment and start a
        // trajectory segment.
        working_move_.demo_states.push_back(demo_state);
        working_move_.target_object = collidee;
        ROS_ASSERT(working_move_.target_object != "");
        segments_->push_back(working_move_);
        working_move_ = NewMoveToSegment();

        working_traj_ = NewTrajSegment();
        working_traj_.target_object = collidee;
        working_traj_.demo_states.push_back(demo_state);
        ROS_INFO("%d: %s transitioning from FREE_GRASP to STATIONARY_COLLISION",
                 index_, arm_name_.c_str());
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
    if (working_traj_.demo_states.size() > 0) {
      ROS_ASSERT(working_traj_.target_object != "");
      segments_->push_back(working_traj_);
    }
    working_traj_ = NewTrajSegment();

    ProgramSegment ungrasp_segment = NewUngraspSegment();
    ungrasp_segment.demo_states.push_back(demo_state);
    segments_->push_back(ungrasp_segment);
    ROS_INFO("%d: %s transitioning from STATIONARY_COLLISION to NONE", index_,
             arm_name_.c_str());
    state_ = NONE;
    return;
  }

  // Check collisions. Either:
  // 1. We exit collision
  // 2. We are in collision
  //    1. We enter double collision
  //    2. We stay in single collision with the same object
  //    3. We enter single collision with a new object (rare)
  const msgs::ObjectState object = GetObjectState(demo_state, hand.object_name);
  std::string collidee(
      collision_checker_.Check(object, demo_state.object_states));
  // Exiting collision
  if (collidee == "") {
    if (working_traj_.demo_states.size() > 0) {
      ROS_ASSERT(working_traj_.target_object != "");
      segments_->push_back(working_traj_);
    }
    working_traj_ = NewTrajSegment();
    working_move_ = NewMoveToSegment();
    working_move_.demo_states.push_back(demo_state);
    ROS_INFO("%d: %s transitioning from STATIONARY_COLLISION to FREE_GRASP",
             index_, arm_name_.c_str());
    state_ = FREE_GRASP;
    return;
  }

  bool is_double_collision =
      other_hand.current_action == msgs::HandState::GRASPING &&
      collidee == other_hand.object_name;
  bool is_same_object = collidee == working_traj_.target_object;
  if (is_double_collision) {
    const std::string target =
        InferDoubleCollisionTarget(demo_states_, index_, collision_checker_);

    // If the target is the same object as before, continue the trajectory.
    // Otherwise, commit this segment and start a new one.
    if (target == working_traj_.target_object) {
      working_traj_.demo_states.push_back(demo_state);
    } else {
      ROS_ASSERT(working_traj_.target_object != "");
      segments_->push_back(working_traj_);
      working_traj_ = NewTrajSegment();
    }

    // Only the master hand actually records the trajectory.
    bool is_master = hand.object_name != target;
    if (is_master) {
      working_traj_.target_object = target;
      working_traj_.demo_states.push_back(demo_state);
    }

    ROS_INFO(
        "%d: %s transitioning from STATIONARY_COLLISION to DOUBLE_COLLISION",
        index_, arm_name_.c_str());
    ROS_INFO("Target object is %s", working_traj_.target_object.c_str());
    state_ = DOUBLE_COLLISION;
  } else {
    if (is_same_object) {
      working_traj_.demo_states.push_back(demo_state);
    } else {
      if (working_traj_.target_object != "") {
        ROS_WARN(
            "Ignoring change in stationary collision from \"%s\" to \"%s\"",
            working_traj_.target_object.c_str(), collidee.c_str());
      }
      /*
      ROS_INFO("Colliding with a different object %s", collidee.c_str());
      if (working_traj_.demo_states.size() > 0) {
        ROS_ASSERT(working_traj_.target_object != "");
        segments_->push_back(working_traj_);
      }
      working_traj_ = NewTrajSegment();
      working_traj_.target_object = collidee;
      working_traj_.demo_states.push_back(demo_state);
      */
    }
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
    bool is_master = working_traj_.target_object != "";
    if (is_master) {
      if (working_traj_.demo_states.size() > 0) {
        segments_->push_back(working_traj_);
        working_traj_ = NewTrajSegment();

        ProgramSegment ungrasp_segment = NewUngraspSegment();
        ungrasp_segment.demo_states.push_back(demo_state);
        segments_->push_back(ungrasp_segment);
      }
    }
    // Regardless of whether this hand is the master or the target, set state to
    // NONE since we ungrasped.
    ROS_INFO("%d: %s transitioning from DOUBLE_COLLISION to NONE", index_,
             arm_name_.c_str());
    state_ = NONE;
  } else if (other_hand.current_action == msgs::HandState::NONE) {
    // If this hand is the master, then continue the trajectory relative to the
    // other object (but transition to STATIONARY COLLISION after). If this hand
    // is the target, then just do nothing.
    bool is_master = working_traj_.target_object != "";
    if (is_master) {
      working_traj_.demo_states.push_back(demo_state);
    }
    // If the other hand ungrasps, then transition to STATIONARY COLLISION
    ROS_INFO(
        "%d: %s transitioning from DOUBLE_COLLISION to STATIONARY_COLLISION",
        index_, arm_name_.c_str());

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
      if (working_traj_.demo_states.size() > 0) {
        ROS_ASSERT(working_traj_.target_object != "");
        segments_->push_back(working_traj_);
        working_traj_ = NewTrajSegment();

        working_move_ = NewMoveToSegment();
        working_move_.demo_states.push_back(demo_state);
      }
      ROS_INFO("%d: %s transitioning from DOUBLE_COLLISION to FREE_GRASP",
               index_, arm_name_.c_str());
      state_ = FREE_GRASP;
    } else if (collidee == other_hand.object_name) {
      // Stay in double collision
      // Update trajectory of master object.
      bool is_master = working_traj_.target_object != "";
      if (is_master) {
        ROS_ASSERT(working_traj_.target_object != "");
        working_traj_.demo_states.push_back(demo_state);
      }
    } else {
      // Stationary collision
      if (working_traj_.demo_states.size() > 0) {
        ROS_ASSERT(working_traj_.target_object != "");
        segments_->push_back(working_traj_);
      }
      working_traj_ = NewTrajSegment();
      working_traj_.target_object = collidee;
      working_traj_.demo_states.push_back(demo_state);
      ROS_INFO(
          "%d: %s transitioning from DOUBLE_COLLISION to STATIONARY_COLLISION",
          index_, arm_name_.c_str());
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
  if (left_length < right_length) {
    return prev_left_obj.name;
  } else {
    return prev_right_obj.name;
  }
}
}  // namespace pbi
