#include <string>
#include <vector>

#include "actionlib/server/simple_action_server.h"
#include "boost/optional.hpp"
#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "task_db/demo_states_db.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/Program.h"
#include "task_utils/bag_utils.h"

#include "task_imitation/program_generator.h"

namespace msgs = task_perception_msgs;
using boost::optional;

namespace pbi {
// Expose an action to generate programs given a bag path
// Plan and retime is as needed
// Then execute it
class ProgramServer {
 public:
  ProgramServer(const DemoStatesDb& demo_states_db);
  void Start();
  void ExecuteImitation(
      const task_perception_msgs::ImitateDemoGoalConstPtr& goal);

 private:
  DemoStatesDb demo_states_db_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<task_perception_msgs::ImitateDemoAction>
      action_server_;
};

ProgramServer::ProgramServer(const DemoStatesDb& demo_states_db)
    : demo_states_db_(demo_states_db),
      nh_(),
      action_server_(
          nh_, "imitate_demo",
          boost::bind(&pbi::ProgramServer::ExecuteImitation, this, _1), false) {
}

void ProgramServer::Start() { action_server_.start(); }

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

  msgs::ImitateDemoResult result;
  action_server_.setSucceeded(result);
}
}  // namespace pbi

int main(int argc, char** argv) {
  ros::init(argc, argv, "program_generator");
  ros::NodeHandle nh;

  const std::string& kDatabaseName("pbi");
  const std::string& kDemoStates("demo_states");
  mongodb_store::MessageStoreProxy demo_states_store(nh, kDemoStates,
                                                     kDatabaseName);
  pbi::DemoStatesDb demo_states_db(&demo_states_store);
  pbi::ProgramServer program_server(demo_states_db);
  program_server.Start();
  ros::spin();
  return 0;
}
