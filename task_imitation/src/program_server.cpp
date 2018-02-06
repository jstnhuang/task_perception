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
#include "ros/ros.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/Program.h"
#include "task_utils/bag_utils.h"
#include "transform_graph/graph.h"

#include "task_imitation/program_generator.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using boost::optional;

namespace pbi {

ProgramIterator::ProgramIterator(const std::vector<msgs::Step>& steps)
    : steps_(steps), step_i_(0), traj_i_(0) {}

void ProgramIterator::Begin() {
  step_i_ = 0;
  traj_i_ = 0;
}

void ProgramIterator::Advance() {
  if (IsDone()) {
    ROS_WARN("Called Advance() on finished ProgramIterator");
    return;
  }
  const msgs::Step& current = step();
  if (current.action_type == msgs::Step::GRASP) {
    ++step_i_;
  } else if (current.action_type == msgs::Step::FOLLOW_TRAJECTORY) {
    if (traj_i_ < current.ee_trajectory.size() - 1) {
      ++traj_i_;
    } else {
      traj_i_ = 0;
      ++step_i_;
    }
  } else if (current.action_type == msgs::Step::UNGRASP) {
    ++step_i_;
  }
}

bool ProgramIterator::IsDone() { return step_i_ >= steps_.size(); }

ros::Duration ProgramIterator::time() {
  ROS_ASSERT(!IsDone());

  const msgs::Step& current = step();
  if (current.action_type == msgs::Step::GRASP) {
    return current.start_time;
  } else if (current.action_type == msgs::Step::FOLLOW_TRAJECTORY) {
    return current.start_time + current.times_from_start[traj_i_];
  } else if (current.action_type == msgs::Step::UNGRASP) {
    return current.start_time;
  } else {
    ROS_ASSERT_MSG(false, "Unsupported action type \"%s\"",
                   current.action_type.c_str());
    ros::Duration zero;
    return zero;
  }
}

msgs::Step ProgramIterator::step() {
  ROS_ASSERT(!IsDone());
  return steps_[step_i_];
}

optional<std::pair<geometry_msgs::Pose, ros::Duration> >
ProgramIterator::trajectory_point() {
  ROS_ASSERT(!IsDone());
  const msgs::Step& current = step();
  if (current.action_type != msgs::Step::FOLLOW_TRAJECTORY) {
    return boost::none;
  }
  return std::make_pair<geometry_msgs::Pose, ros::Duration>(
      current.ee_trajectory[traj_i_], current.times_from_start[traj_i_]);
}

Slice::Slice() : grasp(), left_traj(), right_traj(), ungrasp() {}

void Slice::Reset() {
  msgs::Step blank;
  grasp = blank;
  left_traj = blank;
  right_traj = blank;
  ungrasp = blank;
}

void Slice::FixTrajectories() {
  if (!left_traj.start_time.isZero() && left_traj.ee_trajectory.size() > 0) {
    ros::Duration offset = left_traj.times_from_start[0];
    left_traj.start_time += offset;
    for (size_t i = 0; i < left_traj.times_from_start.size(); ++i) {
      left_traj.times_from_start[i] -= (offset - ros::Duration(0.033));
    }
  }
  if (!right_traj.start_time.isZero() && right_traj.ee_trajectory.size() > 0) {
    ros::Duration offset = right_traj.times_from_start[0];
    right_traj.start_time += offset;
    for (size_t i = 0; i < right_traj.times_from_start.size(); ++i) {
      right_traj.times_from_start[i] -= (offset - ros::Duration(0.033));
    }
  }
}

std::vector<Slice> SliceProgram(const task_perception_msgs::Program& program) {
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
      optional<std::pair<geometry_msgs::Pose, ros::Duration> > pt =
          it->trajectory_point();
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

ProgramServer::ProgramServer(const ros::ServiceClient& db_client,
                             const std::string& moveit_planning_group)
    : db_client_(db_client),
      move_group_(moveit_planning_group),
      nh_(),
      action_server_(
          nh_, "imitate_demo",
          boost::bind(&pbi::ProgramServer::ExecuteImitation, this, _1), false),
      initialize_object_("initialize_object"),
      planning_frame_(move_group_.getPlanningFrame()) {}

void ProgramServer::Start() {
  ROS_INFO("Using planning frame: %s", planning_frame_.c_str());
  action_server_.start();
  while (ros::ok() && !initialize_object_.waitForServer(ros::Duration(2.0))) {
    ROS_WARN("Waiting for object initializer action.");
  }
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
  std::vector<Slice> slices = ComputeSlices(get_states_res.demo_states);

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
    ROS_INFO("Initializing pose for object: \"%s\"", it->first.c_str());
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
  }

  return object_states;
}

std::vector<Slice> ProgramServer::ComputeSlices(
    const msgs::DemoStates& demo_states) {
  ProgramGenerator generator;
  for (size_t i = 0; i < demo_states.demo_states.size(); ++i) {
    generator.Step(demo_states.demo_states[i]);
  }
  msgs::Program program = generator.program();
  std::map<std::string, msgs::ObjectState> object_states =
      GetObjectPoses(program);
  ROS_INFO("All object states initialized.");

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
        const geometry_msgs::Pose& pose = step.ee_trajectory[traj_i];
        tg::Transform pose_in_planning;
        bool success =
            graph.DescribePose(pose, tg::Source(object_name),
                               tg::Target(planning_frame_), &pose_in_planning);
        ROS_ASSERT(success);
        step.ee_trajectory[traj_i] = pose_in_planning.pose();
      }
    }
  }
  return SliceProgram(program);
}
}  // namespace pbi
