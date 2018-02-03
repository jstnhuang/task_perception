// This is super annoying. For some reason, MoveIt and mongodb_store cannot be
// linked together. The executable immediately crashes somewhere inside of
// libwarehouse_ros. So, we need expose a service.

#include <string>

#include "boost/optional.hpp"
#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "task_perception_msgs/GetDemoStates.h"

#include "task_db/demo_states_db.h"

namespace msgs = task_perception_msgs;

namespace pbi {
class DbServer {
 public:
  DbServer(const DemoStatesDb& db);
  bool Get(msgs::GetDemoStatesRequest& req, msgs::GetDemoStatesResponse& res);

 private:
  DemoStatesDb db_;
};

DbServer::DbServer(const DemoStatesDb& db) : db_(db) {}

bool DbServer::Get(msgs::GetDemoStatesRequest& req,
                   msgs::GetDemoStatesResponse& res) {
  boost::optional<std::string> db_id = db_.GetIdByName(req.name);
  if (!db_id) {
    res.error = "Unable to find name \"" + req.name + "\"";
    return true;
  }
  boost::optional<msgs::DemoStates> results = db_.Get(*db_id);
  if (!results) {
    res.error = "Unable to find ID \"" + *db_id + "\"";
    return true;
  }
  res.demo_states = *results;
  return true;
}
}  // namespace pbi

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_states_db_server");
  ros::NodeHandle nh;

  const std::string& kDatabaseName("pbi");
  const std::string& kDemoStates("demo_states");
  mongodb_store::MessageStoreProxy demo_states_store(nh, kDemoStates,
                                                     kDatabaseName);
  pbi::DemoStatesDb db(&demo_states_store);

  pbi::DbServer server(db);
  ros::ServiceServer service =
      nh.advertiseService("get_demo_states", &pbi::DbServer::Get, &server);

  ros::spin();
  return 0;
}
