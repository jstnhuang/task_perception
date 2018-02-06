#ifndef _PBI_PROGRAM_SERVER_H_
#define _PBI_PROGRAM_SERVER_H_

#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "boost/optional.hpp"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "moveit/move_group_interface/move_group.h"
#include "pr2_actions/gripper.h"
#include "ros/ros.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/Program.h"
#include "tf/transform_listener.h"

namespace pbi {
// Iterates through events in a list of steps. An event is either a grasp, an
// ungrasp, or a trajectory point.
class ProgramIterator {
 public:
  explicit ProgramIterator(
      const std::vector<task_perception_msgs::Step>& steps);

  // This must be called to initialize the iterator.
  void Begin();

  // Calling this will advance the iterator to point to the next event.
  void Advance();

  // Returns true if the iterator has advanced past the last event.
  bool IsDone();

  // Gets the time of the current event. For grasp/ungrasp events, this is the
  // start_time. For trajectory points, this is the start_time plus the
  // time_from_start of the point.
  //
  // Do not call if IsDone() is true.
  ros::Duration time();

  // Gets the current step the iterator is pointing to.
  //
  // Do not call if IsDone() is true.
  task_perception_msgs::Step step();

  // Gets the trajectory point the iterator is pointing to, if any.
  //
  // Do not call if IsDone() is true.
  boost::optional<std::pair<geometry_msgs::Pose, ros::Duration> >
  trajectory_point();

 private:
  const std::vector<task_perception_msgs::Step>& steps_;
  size_t step_i_;
  size_t traj_i_;
};

// A Slice represents part of a program.
// A program is simply a list of steps as demonstrated by a person. Each step
// takes some duration to execute. Each step needs to be re-timed to run
// properly on the robot, changing the duration of each step. This may affect
// the synchronization of bimanual motions.
//
// To solve this, we "slice" the program such that a slice starts when an arm
// action begins and ends when an arm action ends. Whatever the other arm is
// doing during the duration of the slice is copied over. We then retime all the
// slices individually.
//
// Example: left arm grasps object from t=0 to 10, right arm grasps object from
// t=6 to 12. Then the slices are: [0-6], [6-10], [10-12].
//
// The grasp/ungrasp steps are filled out depending on whether the slice
// starts/ends with a left/right arm action. Otherwise, they are left blank.
class Slice {
 public:
  Slice();

  // Resets this slice so that all the steps are blank messages.
  void Reset();

  // Shift left and right trajectories such that time_from_start always starts
  // at 0.03. Also adjusts the start_time to match.
  //
  // E.g., if we slice with a trajectory from t=[1, 10] into t=[1, 6] and
  // t=[6,10], then the second trajectory will have a start_time of 1 and its
  // first time_from_start will be 5. After FixTrajectories(), it will have a
  // start time of 6 and a time_from_start of 0.03.
  //
  // TODO: Harded 0.03s start time assumes trajectory data is recorded at 30Hz.
  // We lose this information in the slicing. To regain this information, we
  // need the previous slice.
  void FixTrajectories();

  task_perception_msgs::Step grasp;
  task_perception_msgs::Step left_traj;
  task_perception_msgs::Step right_traj;
  task_perception_msgs::Step ungrasp;
};

std::vector<Slice> SliceProgram(const task_perception_msgs::Program& program);
std::vector<geometry_msgs::Pose> SampleTrajectory(
    const std::vector<geometry_msgs::Pose>& traj);

class ProgramServer {
 public:
  ProgramServer(const ros::ServiceClient& db_client);
  void Start();
  void ExecuteImitation(
      const task_perception_msgs::ImitateDemoGoalConstPtr& goal);

 private:
  std::map<std::string, task_perception_msgs::ObjectState> GetObjectPoses(
      const task_perception_msgs::Program& program);
  std::vector<Slice> ComputeSlices(
      const task_perception_msgs::DemoStates& demo_states);

  ros::ServiceClient db_client_;
  moveit::planning_interface::MoveGroup left_group_;
  moveit::planning_interface::MoveGroup right_group_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<task_perception_msgs::ImitateDemoAction>
      action_server_;
  actionlib::SimpleActionClient<dbot_ros_msgs::InitializeObjectAction>
      initialize_object_;
  const std::string planning_frame_;

  ros::Publisher left_traj_pub_;
  ros::Publisher right_traj_pub_;

  pr2_actions::Gripper left_gripper_;
  pr2_actions::Gripper right_gripper_;
  tf::TransformListener tf_listener_;
};
}  // namespace pbi

#endif  // _PBI_PROGRAM_SERVER_H_
