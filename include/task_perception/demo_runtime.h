#ifndef _PBI_DEMO_RUNTIME_H_
#define _PBI_DEMO_RUNTIME_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/DemoState.h"
#include "task_perception_msgs/ObjectState.h"

#include "task_perception/demo_model.h"
#include "task_perception/demo_visualizer.h"
#include "task_perception/object_tracker.h"
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
// The execution state at frame N depends on the state at frame N-1 and the set
// of events that occur at time N. So, if you have already executed frame N but
// want to change its events, just reset to the state at N-1 and execute the new
// set of events.
//
// runtime.Rewind(5);
// demo_model.DeleteEvent("set skeleton pose", 5);
// runtime.Execute(demo_model.EventsAt(5));
class DemoRuntime {
 public:
  DemoRuntime(const DemoVisualizer& viz, const SkeletonServices& skel_services);

  void LoadDemo(const std::string& color_topic, const std::string& depth_topic,
                const sensor_msgs::CameraInfo& camera_info,
                const std::vector<sensor_msgs::Image>& color_images,
                const std::vector<sensor_msgs::Image>& depth_images,
                const std::shared_ptr<DemoModel>& demo_model);

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
  void Rewind(const int last_executed_frame);

 private:
  void ResetState();
  void PublishViz();

  DemoVisualizer viz_;
  SkeletonServices skel_services_;
  std::shared_ptr<DemoModel> demo_model_;

  ros::NodeHandle nh_;

  // Image state
  VideoScrubber color_scrubber_;
  VideoScrubber depth_scrubber_;
  sensor_msgs::Image current_color_image_;
  sensor_msgs::Image current_depth_image_;
  sensor_msgs::CameraInfo camera_info_;

  // Execution state
  // 0-based index of the last video frame that was executed. -1 if nothing has
  // been executed.
  int last_executed_frame_;
  int num_frames_;
  std::vector<task_perception_msgs::DemoState> states_;
};
}  // namespace pbi

#endif  // _PBI_DEMO_RUNTIME_H_
