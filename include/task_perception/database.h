#ifndef _PBI_DATABASE_H_
#define _PBI_DATABASE_H_

#include <map>
#include <string>

#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "task_perception_msgs/Demonstration.h"

namespace pbi {
class DemonstrationDb {
 public:
  DemonstrationDb(mongodb_store::MessageStoreProxy* db,
                  const ros::Publisher& demo_pub);

  std::string Insert(const task_perception_msgs::Demonstration& demo);
  void Update(const std::string& db_id,
              const task_perception_msgs::Demonstration& demo);
  bool Get(const std::string& db_id,
           task_perception_msgs::Demonstration* demo) const;
  // Returns empty string if not found.
  std::string GetIdByName(const std::string& name) const;
  void Delete(const std::string& db_id);
  void PublishDemonstration(const std::string& db_id);

 private:
  ros::NodeHandle nh_;
  mongodb_store::MessageStoreProxy* db_;
  ros::Publisher demo_pub_;
};
}  // namespace pbi

#endif  // _PBI_DATABASE_H_
