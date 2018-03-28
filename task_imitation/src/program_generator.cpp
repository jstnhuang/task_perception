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
  msgs::DemoState prev_state;
  std::vector<ProgramSegment> segments;
  ProgramSegment clean_l_traj_segment;
  clean_l_traj_segment.arm_name = msgs::Step::LEFT;
  clean_l_traj_segment.type = msgs::Step::FOLLOW_TRAJECTORY;
  ProgramSegment clean_r_traj_segment;
  clean_r_traj_segment.arm_name = msgs::Step::RIGHT;
  clean_r_traj_segment.type = msgs::Step::FOLLOW_TRAJECTORY;
  ProgramSegment l_traj_segment = clean_l_traj_segment;
  ProgramSegment r_traj_segment = clean_r_traj_segment;
  BOOST_FOREACH (const msgs::DemoState& state, demo_states) {
    // Check if we should add grasp or ungrasp events.
    // Left hand
    optional<ProgramSegment> l_grasp_segment =
        SegmentGrasp(state, prev_state, msgs::Step::LEFT);
    if (l_grasp_segment) {
      segments.push_back(*l_grasp_segment);
    }

    optional<ProgramSegment> l_ungrasp_segment =
        SegmentUngrasp(state, prev_state, msgs::Step::LEFT);
    if (l_ungrasp_segment) {
      segments.push_back(*l_ungrasp_segment);
      if (l_traj_segment.demo_states.size() > 0) {
        segments.push_back(l_traj_segment);
      }
      l_traj_segment = clean_l_traj_segment;
    }

    // Right hand
    optional<ProgramSegment> r_grasp_segment =
        SegmentGrasp(state, prev_state, msgs::Step::RIGHT);
    if (r_grasp_segment) {
      segments.push_back(*r_grasp_segment);
    }

    optional<ProgramSegment> r_ungrasp_segment =
        SegmentUngrasp(state, prev_state, msgs::Step::RIGHT);
    if (r_ungrasp_segment) {
      segments.push_back(*r_ungrasp_segment);
      if (r_traj_segment.demo_states.size() > 0) {
        segments.push_back(r_traj_segment);
      }
      r_traj_segment = clean_r_traj_segment;
    }

    bool is_left_grasping =
        state.left_hand.current_action == msgs::HandState::GRASPING;
    bool is_right_grasping =
        state.right_hand.current_action == msgs::HandState::GRASPING;
    if (is_left_grasping || is_right_grasping) {
      // First, check if both hands are holding objects and the two objects are
      // colliding.
      msgs::ObjectState left_obj =
          GetObjectState(state, state.left_hand.object_name);
      std::string left_target = CheckCollisions(left_obj, state.object_states);
      // Check if we are colliding with the object held by the other hand.
      // If so, determine which one is the "target".
      // If we are actually the target, then set the target to "", meaning that
      // we do not record the motion of this hand.
      if (is_right_grasping && left_target == state.right_hand.object_name) {
        // TODO: fill this out
      } else {
        // Otherwise, the target is whichever object is not being held.
        if (is_left_grasping) {
          if (left_target == l_traj_segment.target_object) {
            l_traj_segment.demo_states.push_back(state);
          } else {
            // Either the left_target is a different object, or it is the empty
            // string, indicating that we are no longer in collision. In either
            // case, commit the trajectory and start a new one with
            // target_object = left_target.
            if (l_traj_segment.demo_states.size() > 0) {
              segments.push_back(l_traj_segment);
            }
            l_traj_segment = clean_l_traj_segment;
            l_traj_segment.target_object = left_target;
            l_traj_segment.demo_states.push_back(state);
          }
        }
        if (is_right_grasping) {
          msgs::ObjectState right_obj =
              GetObjectState(state, state.right_hand.object_name);
          std::string right_target =
              CheckCollisions(right_obj, state.object_states);

          // We already checked if the two held objects are colliding above.
          // At this point, we know that the right held object, if it is
          // colliding, is not colliding with the object held in the left hand.
          ROS_ASSERT(right_target != state.left_hand.object_name);

          if (right_target == r_traj_segment.target_object) {
            r_traj_segment.demo_states.push_back(state);
          } else {
            if (r_traj_segment.demo_states.size() > 0) {
              segments.push_back(r_traj_segment);
            }
            r_traj_segment = clean_r_traj_segment;
            r_traj_segment.target_object = right_target;
            r_traj_segment.demo_states.push_back(state);
          }
        }
      }
    }

    prev_state = state;
  }
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

std::string ProgramGenerator::CheckCollisions(
    const task_perception_msgs::ObjectState& object,
    const std::vector<task_perception_msgs::ObjectState>& other_objects) {
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
