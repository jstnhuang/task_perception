#ifndef _PBI_PROGRAM_SERVER_H_
#define _PBI_PROGRAM_SERVER_H_

#include <map>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "moveit/move_group_interface/move_group.h"
#include "rapid_robot/camera_interface.h"
#include "ros/ros.h"
#include "surface_perception/surface_objects.h"
#include "surface_perception/visualization.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/Program.h"

#include "task_imitation/program_executor.h"

namespace pbi {
class ProgramServer {
 public:
  ProgramServer(const ros::ServiceClient& db_client,
                const rapid::PointCloudCameraInterface& cam_interface);
  void Start();
  void ExecuteImitation(
      const task_perception_msgs::ImitateDemoGoalConstPtr& goal);

 private:
  std::map<std::string, task_perception_msgs::ObjectState> GetObjectPoses(
      const task_perception_msgs::DemoStates& demo_states);

  ros::ServiceClient db_client_;
  const rapid::PointCloudCameraInterface& cam_interface_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<task_perception_msgs::ImitateDemoAction>
      action_server_;
  actionlib::SimpleActionClient<dbot_ros_msgs::InitializeObjectAction>
      initialize_object_;

  moveit::planning_interface::MoveGroup left_group_;
  moveit::planning_interface::MoveGroup right_group_;

  ProgramExecutor executor_;

  ros::Publisher program_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher segmentation_pub_;
  surface_perception::SurfaceViz segmentation_viz_;
  LazyObjectModel::ObjectModelCache model_cache_;
  std::string planning_frame_;
};

// Find which object in surface_objects best matches the object whose scale is
// obj_scale. Matches must be "good enough," meaning each dimension matches by
// some amount. If more than one "good enough" matches are found, the best match
// will be returned (as defined by the norm of the difference in scale). Returns
// true if a "good enough" match was found, false otherwise. The pose and scale
// of the best match will be output.
bool MatchObject(
    const geometry_msgs::Vector3& obj_scale,
    const std::vector<surface_perception::SurfaceObjects>& surface_objects,
    geometry_msgs::Pose* pose, geometry_msgs::Vector3* scale);

}  // namespace pbi

#endif  // _PBI_PROGRAM_SERVER_H_
