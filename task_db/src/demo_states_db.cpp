#include "task_db/demo_states_db.h"

#include <string>
#include <vector>

#include "boost/optional.hpp"
#include "boost/shared_ptr.hpp"
#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoStates.h"

using boost::shared_ptr;
using task_perception_msgs::DemoStates;
using std::pair;
using std::vector;

namespace pbi {
DemoStatesDb::DemoStatesDb(mongodb_store::MessageStoreProxy* db) : db_(db) {}

std::string DemoStatesDb::Insert(const DemoStates& demo) {
  return db_->insert(demo);
}

void DemoStatesDb::Update(const std::string& db_id, const DemoStates& demo) {
  bool success = db_->updateID(db_id, demo);
  if (!success) {
    ROS_ERROR("Failed to update demo states with ID: \"%s\"", db_id.c_str());
    return;
  }
}

boost::optional<DemoStates> DemoStatesDb::Get(const std::string& db_id) const {
  vector<shared_ptr<DemoStates> > results;
  bool success = db_->queryID(db_id, results);
  if (!success || results.size() < 1) {
    ROS_INFO("Can't get demo states with ID: \"%s\"", db_id.c_str());
    return boost::none;
  }

  return *results[0];
}

boost::optional<std::string> DemoStatesDb::GetIdByName(
    const std::string& name) const {
  vector<shared_ptr<DemoStates> > results;
  mongo::BSONObj query = BSON("name" << name);
  mongo::BSONObj meta_query;
  mongo::BSONObj sort_query;
  bool find_one = true;
  bool decode_metas = true;
  int limit = 1;

  vector<std::pair<shared_ptr<DemoStates>, mongo::BSONObj> > msg_and_metas;
  bool success = db_->query(msg_and_metas, query, meta_query, sort_query,
                            find_one, decode_metas, limit);
  if (!success || msg_and_metas.size() < 1) {
    ROS_INFO("Can't get demo states with name: \"%s\"", name.c_str());
    return boost::none;
  }
  mongo::BSONObj meta = msg_and_metas[0].second;
  mongo::BSONElement id_elem;
  success = meta.getObjectID(id_elem);
  if (!success) {
    ROS_ERROR("Failed to get object ID!");
    return boost::none;
  }
  return id_elem.OID().toString();
}
}  // namespace pbi
