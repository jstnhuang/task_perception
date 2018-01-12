#ifndef _PBI_ANNOTATOR_SERVER_H_
#define _PBI_ANNOTATOR_SERVER_H_

#include <map>
#include <memory>
#include <string>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/NerfJointStates.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"
#include "task_perception_msgs/Demonstration.h"

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
                  const DemonstrationDb& demo_db);
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
  void HandleDeleteEvent(const std::string& event_type,
                         const std::string& object_name);

  void ProcessCurrentStep();

  // Advances the skeleton tracker with the given color/depth frame.
  void AdvanceSkeleton(const sensor_msgs::Image& color,
                       const sensor_msgs::Image& depth);

  void PublishState();

  DemoVisualizer demo_viz_;
  SkeletonServices skel_services_;
  DemonstrationDb demo_db_;

  ros::NodeHandle nh_;

  // Bag file state
  std::shared_ptr<rosbag::Bag> bag_;
  sensor_msgs::CameraInfo camera_info_;
  std::string demo_id_;
  std::shared_ptr<DemoModel> demo_model_;

  task_perception_msgs::AnnotatorState state_;

  DemoRuntime demo_runtime_;
};
}  // namespace pbi

#endif  // _PBI_ANNOTATOR_SERVER_H_
