#include "task_imitation/program_server.h"

#include <limits.h>
#include <map>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/Program.h"
#include "task_utils/bag_utils.h"
#include "transform_graph/graph.h"

#include "task_imitation/program_executor.h"
#include "task_imitation/program_generator.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using boost::optional;
using geometry_msgs::Pose;

namespace pbi {
ProgramServer::ProgramServer(
    const ros::ServiceClient& db_client,
    const rapid::PointCloudCameraInterface& cam_interface)
    : db_client_(db_client),
      cam_interface_(cam_interface),
      nh_(),
      action_server_(
          nh_, "imitate_demo",
          boost::bind(&pbi::ProgramServer::ExecuteImitation, this, _1), false),
      initialize_object_("initialize_object"),
      left_group_("left_arm"),
      right_group_("right_arm"),
      executor_(left_group_, right_group_),
      program_pub_(
          nh_.advertise<msgs::Program>("pbi_imitation/program", 1, true)) {}

void ProgramServer::Start() {
  action_server_.start();
  while (ros::ok() && !initialize_object_.waitForServer(ros::Duration(2.0))) {
    ROS_WARN("Waiting for object initializer action.");
  }
  executor_.Init();
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
  std::map<std::string, msgs::ObjectState> object_states =
      GetObjectPoses(demo_states);
  ROS_INFO("All object states initialized.");

  ProgramGenerator generator(left_group_, right_group_);
  msgs::Program program =
      generator.Generate(demo_states.demo_states, object_states);
  program_pub_.publish(program);

  executor_.Execute(program, object_states);
  msgs::ImitateDemoResult result;
  action_server_.setSucceeded(result);
}

std::map<std::string, msgs::ObjectState> ProgramServer::GetObjectPoses(
    const msgs::DemoStates& demo_states) {
  // This models the assumption that each demonstration only interacts with an
  // object once.We could / should allow the robot to interact with an object
  // more than once, but we need to continuosly track the objects in that
  // case.
  std::map<std::string, msgs::ObjectState> object_states;
  for (size_t i = 0; i < demo_states.demo_states.size(); ++i) {
    const msgs::DemoState& state = demo_states.demo_states[i];
    for (size_t j = 0; j < state.object_states.size(); ++j) {
      const msgs::ObjectState& os = state.object_states[j];
      if (object_states.find(os.name) == object_states.end()) {
        object_states[os.name] = os;
      }
    }
  }

  for (std::map<std::string, msgs::ObjectState>::iterator it =
           object_states.begin();
       it != object_states.end(); ++it) {
    dbot_ros_msgs::InitializeObjectGoal init_goal;
    init_goal.frame_id = executor_.planning_frame();
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
}  // namespace pbi
