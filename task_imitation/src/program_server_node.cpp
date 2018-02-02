#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "task_db/demo_states_db.h"

#include "task_imitation/program_server.h"

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
