#ifndef _PBI_CONTACT_DETECTION_H_
#define _PBI_CONTACT_DETECTION_H_

#include <string>

#include "pcl/kdtree/kdtree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "task_perception_msgs/DemoState.h"

#include "task_perception/grasp_planner.h"
#include "task_perception/task_perception_context.h"
namespace pbi {

class ContactDetection {
 public:
  ContactDetection();

  void Predict(TaskPerceptionContext* context,
               task_perception_msgs::HandState* left_hand,
               task_perception_msgs::HandState* right_hand);

 private:
  // Predict the state of a hand given its previous state and other context.
  void PredictHandState(const task_perception_msgs::HandState& prev_state,
                        const std::string& left_or_right,
                        TaskPerceptionContext* context,
                        task_perception_msgs::HandState* hand_state);

  // Check if we should transition from NONE -> GRASPING.
  void CheckGrasp(const task_perception_msgs::HandState& prev_state,
                  const std::string& left_or_right,
                  TaskPerceptionContext* context,
                  task_perception_msgs::HandState* hand_state);

  // Check if we should transition from [GRASPING, PUSHING] -> NONE.
  void CheckRelease(const task_perception_msgs::HandState& prev_state,
                    const std::string& left_or_right,
                    TaskPerceptionContext* context,
                    task_perception_msgs::HandState* hand_state);

  bool IsObjectCurrentlyCloseToWrist(const geometry_msgs::Pose& wrist,
                                     const std::string& object_name,
                                     TaskPerceptionContext* context) const;

  bool IsObjectMoving(const task_perception_msgs::ObjectState& object,
                      TaskPerceptionContext* context) const;

  int NumHandPointsOnObject(const task_perception_msgs::ObjectState& object,
                            const std::string& left_or_right,
                            TaskPerceptionContext* context,
                            const float distance_threshold) const;

  void PublishWristPoses(const geometry_msgs::Pose& left,
                         const geometry_msgs::Pose& right,
                         const std::string& frame_id);

  ros::NodeHandle nh_;
  // Internal visualizations for debugging purposes
  ros::Publisher viz_;
  ros::Publisher obj_viz_;
  ros::Publisher left_hand_viz_;
  ros::Publisher right_hand_viz_;

  GraspPlanner grasp_planner_;
};
}  // namespace pbi

#endif  // _PBI_CONTACT_DETECTION_H_
