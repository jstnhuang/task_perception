#ifndef _PBI_DEMO_RUNTIME_H_
#define _PBI_DEMO_RUNTIME_H_

#include <string>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ObjectState.h"

#include "task_perception/contact_detection.h"
#include "task_perception/demo_model.h"
#include "task_perception/demo_visualizer.h"
#include "task_perception/multi_object_tracker.h"
#include "task_perception/skeleton_services.h"
#include "task_perception/video_scrubber.h"

namespace pbi {
// DemoRuntime simulates the state changes that occur in a demonstration.
//
// DemoRuntime stores the state of the task demonstration for each video frame
// and simulates the effects of demonstration Events on the next state. Users
// can use DemoRuntime to start execution at any state.
//
// Usage:
//   DemoRuntime runtime(demo_model);
//   for (int i=0; i<demo_model.num_frames(); ++i) {
//     ROS_INFO("Evaluating step: %d", runtime.current_frame_number());
//     runtime.Step();
//   }
//
// You can also model the most recently executed step and re-execute it:
//   runtime.RerunLastFrame();
// If the modification you are making is to remove a SPAWN or UNSPAWN event,
// then you must call RemoveSpawnObjectEvent or RemoveUnspawnObjectEvent before
// calling RerunLastFrame.
class DemoRuntime {
 public:
  DemoRuntime(const DemoVisualizer& viz, const SkeletonServices& skel_services,
              const ros::ServiceClient& predict_hands,
              const MultiObjectTracker& object_trackers);

  void LoadDemo(const std::string& color_topic, const std::string& depth_topic,
                const sensor_msgs::CameraInfo& camera_info,
                const std::vector<sensor_msgs::Image>& color_images,
                const std::vector<sensor_msgs::Image>& depth_images,
                const boost::shared_ptr<DemoModel>& demo_model);

  // Evaluates the state at the current frame given the previous state and the
  // events for the current frame.
  void Step();

  // Returns frame number, which is the index of the earliest frame that has not
  // yet been evaluated.
  int last_executed_frame() const;
  void current_color_image(sensor_msgs::Image* image) const;
  void current_depth_image(sensor_msgs::Image* image) const;

  void GetState(const int frame_number,
                task_perception_msgs::DemoState* state) const;
  bool GetObjectState(const int frame_number, const std::string& object_name,
                      task_perception_msgs::ObjectState* object_state);

  // Rewind the runtime. The given frame number specifies the last executed
  // frame, or -1 to start over from the beginning.
  void RerunLastFrame();
  void RemoveSpawnObjectEvent(const std::string& object_name);
  void RemoveUnspawnObjectEvent(const std::string& object_name,
                                const std::string& object_mesh);
  // TODO: If the user removes a SET_OBJECT_POSE event, we should reset the
  // tracker to the previous state's pose. However, we don't keep track of the
  // object's velocity, so the object tracking state gets messed up either way.
  // To get around it, the user can just re-run the annotation from the
  // beginning.
  // void RemoveObjectPoseEvent();

 private:
  // Given the previous state, steps through the skeleton tracker if needed.
  void StepSkeleton(const int frame_number,
                    const task_perception_msgs::DemoState& prev_state,
                    skin_segmentation_msgs::NerfJointStates* nerf_joint_states);
  // Given the previous state, steps through SPAWN and UNSPAWN events.
  // This involves adding or removing object trackers as needed.
  void StepSpawnUnspawn(const int frame_number,
                        const task_perception_msgs::DemoState& prev_state);
  void StepObjectPose(
      const int frame_number, const task_perception_msgs::DemoState& prev_state,
      std::vector<task_perception_msgs::ObjectState>* object_states);

  void ResetState();
  void PublishViz();

  DemoVisualizer viz_;
  SkeletonServices skel_services_;
  ros::ServiceClient predict_hands_;
  MultiObjectTracker object_trackers_;

  boost::shared_ptr<DemoModel> demo_model_;
  ros::NodeHandle nh_;

  // Image state
  VideoScrubber color_scrubber_;
  VideoScrubber depth_scrubber_;
  sensor_msgs::Image current_color_image_;
  sensor_msgs::Image current_depth_image_;
  sensor_msgs::CameraInfo camera_info_;

  // Object model cache
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> object_models_;
  ContactDetection contact_detection_;

  // Execution state
  // 0-based index of the last video frame that was executed. -1 if nothing has
  // been executed.
  int last_executed_frame_;
  int num_frames_;
  std::vector<task_perception_msgs::DemoState> states_;
};
}  // namespace pbi

#endif  // _PBI_DEMO_RUNTIME_H_
