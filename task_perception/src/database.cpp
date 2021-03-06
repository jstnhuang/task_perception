#include "task_perception/database.h"

#include <string>
#include <vector>

#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/Demonstration.h"
#include "task_perception_msgs/Event.h"

using boost::shared_ptr;
using task_perception_msgs::Demonstration;
using std::pair;
using std::vector;

namespace pbi {
DemonstrationDb::DemonstrationDb(mongodb_store::MessageStoreProxy* db,
                                 const ros::Publisher& demo_pub)
    : nh_(), db_(db), demo_pub_(demo_pub) {}

std::string DemonstrationDb::Insert(const Demonstration& demo) {
  std::string id = db_->insert(demo);
  PublishDemonstration(id);
  return id;
}

void DemonstrationDb::Update(const std::string& db_id,
                             const Demonstration& demo) {
  bool success = db_->updateID(db_id, demo);
  if (!success) {
    ROS_ERROR("Failed to update demonstration with ID: \"%s\"", db_id.c_str());
    return;
  }
  PublishDemonstration(db_id);
}

bool DemonstrationDb::Get(const std::string& db_id, Demonstration* demo) const {
  vector<shared_ptr<Demonstration> > results;
  bool success = db_->queryID(db_id, results);
  if (!success || results.size() < 1) {
    ROS_INFO("Can't get demonstration with ID: \"%s\"", db_id.c_str());
    return false;
  }
  *demo = *results[0];
  return true;
}

std::string DemonstrationDb::GetIdByName(const std::string& name) const {
  vector<shared_ptr<Demonstration> > results;
  mongo::BSONObj query = BSON("name" << name);
  mongo::BSONObj meta_query;
  mongo::BSONObj sort_query;
  bool find_one = true;
  bool decode_metas = true;
  int limit = 1;

  vector<std::pair<shared_ptr<Demonstration>, mongo::BSONObj> > msg_and_metas;
  bool success = db_->query(msg_and_metas, query, meta_query, sort_query,
                            find_one, decode_metas, limit);
  if (!success || msg_and_metas.size() < 1) {
    ROS_INFO("Can't get demonstration with name: \"%s\"", name.c_str());
    return "";
  }
  mongo::BSONObj meta = msg_and_metas[0].second;
  mongo::BSONElement id_elem;
  success = meta.getObjectID(id_elem);
  if (!success) {
    ROS_ERROR("Failed to get object ID!");
    return "";
  }
  return id_elem.OID().toString();
}

void DemonstrationDb::Delete(const std::string& db_id) {
  bool success = db_->deleteID(db_id);

  if (!success) {
    ROS_ERROR("Could not delete demonstration with ID \"%s\"", db_id.c_str());
  }
}

void DemonstrationDb::PublishDemonstration(const std::string& db_id) {
  vector<shared_ptr<Demonstration> > results;
  bool success = db_->queryID(db_id, results);
  if (!success || results.size() < 1) {
    ROS_ERROR("Could not republish program with ID: \"%s\"", db_id.c_str());
    return;
  }
  demo_pub_.publish(results[0]);
}
}  // namespace pbi
