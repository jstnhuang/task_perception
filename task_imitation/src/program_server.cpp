#include "task_imitation/program_server.h"

#include <map>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"
#include "task_db/demo_states_db.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_utils/bag_utils.h"

#include "task_imitation/program_generator.h"

namespace msgs = task_perception_msgs;
using boost::optional;

namespace pbi {
ProgramServer::ProgramServer(const DemoStatesDb& demo_states_db,
                             const std::string& moveit_planning_group)
    : demo_states_db_(demo_states_db),
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
  std::string name = GetNameFromBagPath(goal->bag_path);
  optional<std::string> db_id = demo_states_db_.GetIdByName(name);
  if (!db_id) {
    msgs::ImitateDemoResult result;
    action_server_.setAborted(result, "Unable to find demonstration " + name);
    return;
  }
  optional<msgs::DemoStates> opt_demo_states = demo_states_db_.Get(*db_id);
  if (!opt_demo_states) {
    msgs::ImitateDemoResult result;
    action_server_.setAborted(result, "Unable to load demonstration " + name);
    return;
  }
  const std::vector<msgs::DemoState>& demo_states =
      opt_demo_states->demo_states;

  ROS_INFO("Generating demonstration...");
  ProgramGenerator generator;
  for (size_t i = 0; i < demo_states.size(); ++i) {
    generator.Step(demo_states[i]);
  }
  msgs::Program program = generator.program();

  ROS_INFO_STREAM("program: " << program);

  // This models the assumption that each demonstration only interacts with
  // an object once. We could/should allow the robot to interact with an object
  // more than once, but we need to continuosly track the objects in that case.
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

  for (std::map<std::string, msgs::ObjectState>::iterator it =
           object_states.begin();
       it != object_states.end(); ++it) {
    ROS_INFO_STREAM("Pose for " << it->first << ": " << it->second.pose);
  }

  msgs::ImitateDemoResult result;
  action_server_.setSucceeded(result);
}
}  // namespace pbi
