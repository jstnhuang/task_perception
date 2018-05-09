#ifndef _PBI_PROGRAM_SERVER_H_
#define _PBI_PROGRAM_SERVER_H_

#include <map>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group.h"
#include "rapid_manipulation/moveit_planning_scene.h"
#include "rapid_robot/camera_interface.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "surface_perception/visualization.h"
#include "task_perception/object_model_cache.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GenerateProgramAction.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/ImitationEvent.h"
#include "task_perception_msgs/Program.h"
#include "task_utils/pr2_gripper_viz.h"
#include "visualization_msgs/MarkerArray.h"

#include "task_imitation/obb.h"
#include "task_imitation/program_executor.h"

namespace pbi {
class ProgramServer {
 public:
  ProgramServer(const ros::ServiceClient& db_client,
                const rapid::PointCloudCameraInterface& cam_interface);
  void Start();
  void GenerateProgram(
      const task_perception_msgs::GenerateProgramGoalConstPtr& goal);
  void ExecuteImitation(
      const task_perception_msgs::ImitateDemoGoalConstPtr& goal);
  void HandleEvent(const task_perception_msgs::ImitationEvent& event);

 private:
  // Returns error message or ""
  std::string GenerateProgramInternal(
      const std::string& bag_path, task_perception_msgs::Program* program,
      std::map<std::string, task_perception_msgs::ObjectState>* object_states);

  void GetObjectPoses(
      const task_perception_msgs::DemoStates& demo_states,
      std::map<std::string, task_perception_msgs::ObjectState>* object_states,
      Obb* table);
  void VisualizeStep(const task_perception_msgs::Step& step);
  void VisualizeSlice(const task_perception_msgs::ProgramSlice& slice);

  ros::ServiceClient db_client_;
  const rapid::PointCloudCameraInterface& cam_interface_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<task_perception_msgs::GenerateProgramAction>
      generate_program_server_;
  actionlib::SimpleActionServer<task_perception_msgs::ImitateDemoAction>
      imitate_demo_server_;
  actionlib::SimpleActionClient<dbot_ros_msgs::InitializeObjectAction>
      initialize_object_;

  rapid::MoveItPlanningScene planning_scene_;
  moveit::planning_interface::MoveGroup left_group_;
  moveit::planning_interface::MoveGroup right_group_;

  ProgramExecutor executor_;

  ros::Publisher program_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher segmentation_pub_;
  surface_perception::SurfaceViz segmentation_viz_;
  ObjectModelCache model_cache_;
  std::string planning_frame_;

  // Most recent program and object states, used for visualization
  task_perception_msgs::Program program_;
  std::map<std::string, task_perception_msgs::ObjectState> object_states_;

  // Program visualizers
  visualization_msgs::MarkerArray marker_arr_;
  ros::Publisher marker_pub_;
  ros::Publisher traj_pub_;
  Pr2GripperViz gripper_viz_;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_SERVER_H_
