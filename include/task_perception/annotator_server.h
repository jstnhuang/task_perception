#ifndef _PBI_ANNOTATOR_SERVER_H_
#define _PBI_ANNOTATOR_SERVER_H_

#include <map>
#include <memory>
#include <string>

#include "dbot_ros/object_tracker_publisher.h"
#include "dbot_ros/util/interactive_marker_initializer.h"
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
#include "task_perception/particle_tracker_builder.h"
#include "task_perception/video_scrubber.h"

namespace pbi {
class AnnotatorServer {
 public:
  AnnotatorServer(const ros::Publisher& camera_info_pub,
                  const ros::Publisher& color_pub,
                  const ros::Publisher& depth_pub,
                  const ros::Publisher& state_pub,
                  const ros::Publisher& nerf_pub,
                  const DemonstrationDb& demo_db);
  void Start();
  void HandleEvent(const task_perception_msgs::AnnotatorEvent& event);

 private:
  void HandleOpen(const std::string& bag_path);
  void HandleStep();
  void HandleSaveSkeleton();
  void HandleAdvanceSkeleton();
  void HandleAddObject(const std::string& mesh_name);

  void ProcessCurrentStep();

  // Advances the skeleton tracker with the given color/depth frame.
  void AdvanceSkeleton(const sensor_msgs::Image& color,
                       const sensor_msgs::Image& depth);

  // Loop that continuously publishes the RGBD image. This is needed for the
  // depthcloud_encoder node.
  void Loop(const ros::TimerEvent& event);
  void PublishState();

  ros::Publisher camera_info_pub_;
  ros::Publisher color_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher state_pub_;
  ros::Publisher nerf_pub_;
  DemonstrationDb demo_db_;

  ros::NodeHandle nh_;
  ros::Timer timer_;

  // Bag file state
  std::shared_ptr<rosbag::Bag> bag_;
  std::string color_topic_;
  std::string depth_topic_;
  sensor_msgs::CameraInfo camera_info_;
  std::string demo_id_;
  std::shared_ptr<DemoModel> demo_model_;

  task_perception_msgs::AnnotatorState state_;

  VideoScrubber color_scrubber_;
  VideoScrubber depth_scrubber_;
  sensor_msgs::Image current_color_image_;
  sensor_msgs::Image current_depth_image_;

  // Skeleton tracker
  ros::ServiceClient reset_skeleton;
  ros::ServiceClient advance_skeleton;
  ros::ServiceClient get_skeleton_state;

  // Object tracking
  std::shared_ptr<dbot::ObjectTrackerRos<dbot::ParticleTracker> >
      object_tracker_;
  std::shared_ptr<dbot::ObjectStatePublisher> object_pub_;
  std::shared_ptr<opi::InteractiveMarkerInitializer> object_init_;
};

// Returns an initial skeleton that works well for the PR2 at full height and
// camera at 45 degrees.
skin_segmentation_msgs::NerfJointStates DefaultSkeleton();
}  // namespace pbi

#endif  // _PBI_ANNOTATOR_SERVER_H_
