#ifndef _PBI_DEMO_STATES_DB_H_
#define _PBI_DEMO_STATES_DB_H_

#include "boost/optional.hpp"
#include "mongodb_store/message_store.h"
#include "task_perception_msgs/DemoStates.h"

namespace pbi {
class DemoStatesDb {
 public:
  DemoStatesDb(mongodb_store::MessageStoreProxy* db);

  std::string Insert(const task_perception_msgs::DemoStates& demo);

  void Update(const std::string& db_id,
              const task_perception_msgs::DemoStates& demo);

  boost::optional<task_perception_msgs::DemoStates> Get(
      const std::string& db_id) const;

  boost::optional<std::string> GetIdByName(const std::string& name) const;

 private:
  mongodb_store::MessageStoreProxy* db_;
};
}  // namespace pbi

#endif  // _PBI_DEMO_STATES_DB_H_
