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

  std::string Insert(const std::string& name);
  void Update(const std::string& db_id,
              const task_perception_msgs::Demonstration& program);
  bool Get(const std::string& db_id,
           task_perception_msgs::Demonstration* program) const;
  bool GetByName(const std::string& name,
                 task_perception_msgs::Demonstration* program) const;
  void Delete(const std::string& db_id);
  void PublishDemonstration(const std::string& db_id);

 private:
  ros::NodeHandle nh_;
  mongodb_store::MessageStoreProxy* db_;
  ros::Publisher demo_pub_;
};
}  // namespace pbi

#endif  // _PBI_DATABASE_H_
