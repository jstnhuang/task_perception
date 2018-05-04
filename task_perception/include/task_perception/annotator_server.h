#ifndef _PBI_ANNOTATOR_SERVER_H_
#define _PBI_ANNOTATOR_SERVER_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "boost/shared_ptr.hpp"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/NerfJointStates.h"
#include "task_db/demo_states_db.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"

#include "task_perception/database.h"
#include "task_perception/demo_model.h"
#include "task_perception/demo_runtime.h"
#include "task_perception/demo_visualizer.h"
#include "task_perception/skeleton_services.h"

namespace pbi {
// AnnotatorServer edits the annotation in response to AnnotationEvents from the
// frontend.
class AnnotatorServer {
 public:
  AnnotatorServer(const DemoVisualizer& demo_viz,
                  const SkeletonServices& skel_services,
                  const DemonstrationDb& demo_db,
                  const DemoStatesDb& demo_states_db,
                  const ros::ServiceClient& predict_hands,
                  const MultiObjectTracker& object_trackers);
  void Start();
  void HandleEvent(const task_perception_msgs::AnnotatorEvent& event);

 private:
  void HandleOpen(const std::string& bag_path);
  void HandleStep();
  void HandleAddObject(const std::string& object_name,
                       const std::string& mesh_name);
  void HandleRemoveObject(const std::string& object_name);
  void HandleSaveSkeleton();
  void HandleAdvanceSkeleton();
  void HandleDeleteEvent(const task_perception_msgs::AnnotatorEvent& event);
  void HandleSetObjectPose(const std::string& object_name);

  void RunCurrentStep();
  void RerunCurrentStep();

  // Advances the skeleton tracker with the given color/depth frame.
  void AdvanceSkeleton(const sensor_msgs::Image& color,
                       const sensor_msgs::Image& depth);
  bool SetObjectPose(const std::string& object_name,
                     const std::string& object_mesh, geometry_msgs::Pose* pose);

  void PublishState();

  DemoVisualizer demo_viz_;
  SkeletonServices skel_services_;
  DemonstrationDb demo_db_;
  DemoStatesDb demo_states_db_;

  ros::NodeHandle nh_;

  // Bag file state
  boost::shared_ptr<rosbag::Bag> bag_;
  sensor_msgs::CameraInfo camera_info_;
  std::string demo_id_;
  boost::shared_ptr<DemoModel> demo_model_;

  task_perception_msgs::AnnotatorState state_;

  DemoRuntime demo_runtime_;

  actionlib::SimpleActionClient<dbot_ros_msgs::InitializeObjectAction>
      initialize_object_;
};
}  // namespace pbi

#endif  // _PBI_ANNOTATOR_SERVER_H_
