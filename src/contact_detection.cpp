#include "task_perception/contact_detection.h"

#include <math.h>
#include <string>
#include <vector>

#include "pcl/kdtree/kdtree.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "task_perception_msgs/DemoState.h"
#include "visualization_msgs/Marker.h"

#include "task_perception/pcl_typedefs.h"
#include "task_perception/pcl_utils.h"
#include "task_perception/pose_utils.h"
#include "task_perception/task_perception_context.h"

using std::vector;
using std::string;
namespace msgs = task_perception_msgs;

namespace pbi {
ContactDetection::ContactDetection()
    : nh_(),
      viz_(nh_.advertise<visualization_msgs::Marker>(
          "contact_detection/markers", 10)),
      obj_viz_(nh_.advertise<sensor_msgs::PointCloud2>(
          "contact_detection/object_clouds", 1, true)),
      left_hand_viz_(nh_.advertise<sensor_msgs::PointCloud2>(
          "contact_detection/left_hand", 1, true)),
      right_hand_viz_(nh_.advertise<sensor_msgs::PointCloud2>(
          "contact_detection/right_hand", 1, true)) {}

void ContactDetection::Predict(TaskPerceptionContext* context,
                               msgs::HandState* left_hand,
                               msgs::HandState* right_hand) {
  if (!context->LoadParams()) {
    return;
  }

  const msgs::DemoState& prev_state = context->prev_state();
  PredictHandState(prev_state.left_hand, "left", context, left_hand);
  PredictHandState(prev_state.right_hand, "right", context, right_hand);

  if (context->kDebug) {
    geometry_msgs::Pose left_wrist = context->GetLeftWristPose();
    geometry_msgs::Pose right_wrist = context->GetRightWristPose();
    PublishWristPoses(left_wrist, right_wrist,
                      context->camera_info().header.frame_id);
  }

  // Create point cloud for hands
  if (context->kDebug) {
    PointCloudP::Ptr hand_cloud = context->BothHandsCloud();
    PublishPointCloud(left_hand_viz_, hand_cloud, context->LeftHandIndices());
    PublishPointCloud(right_hand_viz_, hand_cloud, context->RightHandIndices());
  }
}

void ContactDetection::PredictHandState(const msgs::HandState& prev_state,
                                        const std::string& left_or_right,
                                        TaskPerceptionContext* context,
                                        msgs::HandState* hand_state) {
  if (prev_state.current_action == "" ||
      prev_state.current_action == msgs::HandState::NONE) {
    CheckGrasp(prev_state, left_or_right, context, hand_state);
  } else if (prev_state.current_action == msgs::HandState::GRASPING) {
    CheckRelease(prev_state, left_or_right, context, hand_state);
  }
}

void ContactDetection::CheckGrasp(const msgs::HandState& prev_state,
                                  const std::string& left_or_right,
                                  TaskPerceptionContext* context,
                                  msgs::HandState* hand_state) {
  // For all nearby objects:
  // - If the object is moving, set state to GRASPING
  // - If enough object points are close to hand points, set state to GRASPING
  // Otherwise, keep state as NONE
  geometry_msgs::Pose wrist_pose;
  if (left_or_right == "left") {
    wrist_pose = context->GetLeftWristPose();
  } else {
    wrist_pose = context->GetRightWristPose();
  }

  vector<msgs::ObjectState> current_objects = context->GetCurrentObjects();
  int num_touched_points = 0;
  for (size_t i = 0; i < current_objects.size(); ++i) {
    const msgs::ObjectState& object = current_objects[i];
    PointCloudP::Ptr object_cloud = context->GetObjectCloud(object.name);
    PublishPointCloud(obj_viz_, *object_cloud);

    if (!IsObjectCurrentlyCloseToWrist(wrist_pose, object.name, context)) {
      continue;
    }

    bool is_moving = IsObjectMoving(object, context);

    bool is_touching = false;
    if (context->BothHandsCloud()->size() == 0) {
      continue;
    }
    int num_touching_points = NumHandPointsOnObject(
        object, left_or_right, context, context->kTouchingObjectDistance);
    is_touching = num_touching_points >= context->kTouchingObjectPoints;

    if (is_moving || is_touching) {
      if (num_touching_points <= num_touched_points) {
        continue;
      }
      num_touched_points = num_touching_points;

      ROS_INFO("Changed %s hand state to GRASPING %s", left_or_right.c_str(),
               object.name.c_str());
      hand_state->current_action = msgs::HandState::GRASPING;
      hand_state->object_name = object.name;
      num_touched_points = num_touching_points;

      grasp_planner_.Plan(left_or_right, object.name, context,
                          &hand_state->contact_pose);
    }
  }
  if (hand_state->current_action == "") {
    hand_state->current_action = msgs::HandState::NONE;
  }
}

void ContactDetection::CheckRelease(const msgs::HandState& prev_state,
                                    const std::string& left_or_right,
                                    TaskPerceptionContext* context,
                                    msgs::HandState* hand_state) {
  // If object does not exist anymore, set state to NONE
  msgs::ObjectState object;
  if (!context->GetCurrentObject(prev_state.object_name, &object)) {
    ROS_WARN("Object \"%s\" disappeared while being contacted by %s hand.",
             prev_state.object_name.c_str(), left_or_right.c_str());
    hand_state->current_action = msgs::HandState::NONE;
    hand_state->object_name = "";
    // TODO: clear contact transform
    return;
  }

  // If not enough object points are close to hand points, set state to NONE
  // When we check for a release, we use more conservative parameters, since
  // sometimes the hand segmentation can flicker or the hand can be occluded
  // while grasping.
  if (context->BothHandsCloud()->size() == 0) {
    ROS_WARN(
        "Hands not found! Maintaining status quo, but this should not happen.");
    *hand_state = prev_state;
    return;
  }
  int num_touching_points = NumHandPointsOnObject(
      object, left_or_right, context, context->kTouchingReleasedObjectDistance);
  if (num_touching_points <= context->kTouchingReleasedObjectPoints) {
    ROS_INFO("Changed %s hand state to NONE (%d out of %d points)",
             left_or_right.c_str(), num_touching_points,
             context->kTouchingReleasedObjectPoints);
    hand_state->current_action = msgs::HandState::NONE;
    hand_state->object_name = "";
    // TODO: clear contact transform
    return;
  }

  // Otherwise, keep as GRASPING
  hand_state->current_action = msgs::HandState::GRASPING;
  hand_state->object_name = prev_state.object_name;
  // TODO: fill in contact transform
  return;
}

bool ContactDetection::IsObjectCurrentlyCloseToWrist(
    const geometry_msgs::Pose& wrist, const std::string& object_name,
    TaskPerceptionContext* context) const {
  KdTreeP::Ptr object_tree = context->GetObjectTree(object_name);

  PointP wrist_point;
  wrist_point.x = wrist.position.x;
  wrist_point.y = wrist.position.y;
  wrist_point.z = wrist.position.z;
  vector<int> indices(1);
  vector<float> sq_distances(1);
  object_tree->nearestKSearch(wrist_point, 1, indices, sq_distances);
  float wrist_threshold =
      context->kCloseToWristDistance * context->kCloseToWristDistance;
  return sq_distances[0] <= wrist_threshold;
}

int ContactDetection::NumHandPointsOnObject(
    const task_perception_msgs::ObjectState& object,
    const std::string& left_or_right, TaskPerceptionContext* context,
    const float distance_threshold) const {
  KdTreeP::Ptr object_tree = context->GetObjectTree(object.name);

  PointCloudP::Ptr both_hands_cloud = context->BothHandsCloud();
  pcl::IndicesPtr hand_indices;
  if (left_or_right == "left") {
    hand_indices = context->LeftHandIndices();
  } else {
    hand_indices = context->RightHandIndices();
  }

  int num_touching = 0;
  vector<int> indices(1);
  vector<float> sq_distances(1);
  const float kSquaredTouchingDistance =
      distance_threshold * distance_threshold;

  const bool kDebugDistances = context->kDebug && false;
  int histogram[5];
  if (kDebugDistances) {
    histogram[0] = 0;
    histogram[1] = 0;
    histogram[2] = 0;
    histogram[3] = 0;
    histogram[4] = 0;
  }
  for (size_t index_i = 0; index_i < hand_indices->size(); ++index_i) {
    int index = (*hand_indices)[index_i];
    object_tree->nearestKSearch(both_hands_cloud->points[index], 1, indices,
                                sq_distances);
    float sq_distance = sq_distances[0];
    if (sq_distance < kSquaredTouchingDistance) {
      num_touching += 1;
    }

    if (kDebugDistances) {
      float distance = sqrt(sq_distance);
      if (distance < 0.002) {
        histogram[0] += 1;
      }
      if (distance < 0.005) {
        histogram[1] += 1;
      }
      if (distance < 0.008) {
        histogram[2] += 1;
      }
      if (distance < 0.01) {
        histogram[3] += 1;
      }
      if (distance < 0.012) {
        histogram[4] += 1;
      }
    }
  }
  if (kDebugDistances) {
    ROS_INFO("%d %d %d %d %d", histogram[0], histogram[1], histogram[2],
             histogram[3], histogram[4]);
  }
  return num_touching;
}

bool ContactDetection::IsObjectMoving(const msgs::ObjectState& object,
                                      TaskPerceptionContext* context) const {
  msgs::ObjectState prev_obj;
  if (!context->GetPreviousObject(object.name, &prev_obj)) {
    return false;
  }

  ros::Duration dt = context->GetCurrentTime() - context->GetPreviousTime();

  // TODO: we only take linear movement into account, but not angular motion
  // Angular motion may be hard to track.
  double linear_distance = LinearDistance(prev_obj.pose, object.pose);
  // double linear_speed = linear_distance / dt.toSec();
  // ROS_INFO("%s: moved %f in %f seconds (%f m/s)", object.name.c_str(),
  //         linear_distance, dt.toSec(), linear_speed);
  return linear_distance >= context->kMovingObjectDistance;
}

void ContactDetection::PublishWristPoses(const geometry_msgs::Pose& left,
                                         const geometry_msgs::Pose& right,
                                         const string& frame_id) {
  visualization_msgs::Marker left_marker;
  left_marker.header.frame_id = frame_id;
  left_marker.ns = "left_wrist";
  left_marker.id = 0;
  left_marker.type = visualization_msgs::Marker::ARROW;
  left_marker.pose = left;
  left_marker.scale.x = 0.1;
  left_marker.scale.y = 0.01;
  left_marker.scale.z = 0.01;
  left_marker.color.r = 1;
  left_marker.color.a = 1;
  viz_.publish(left_marker);

  visualization_msgs::Marker right_marker = left_marker;
  right_marker.ns = "right_wrist";
  right_marker.pose = right;
  viz_.publish(right_marker);

  // Compute z-axis pose and publish.
  // geometry_msgs::Quaternion z_axis;
  // z_axis.w = 0.707;
  // z_axis.y = 0.707;
  // tg::Graph graph;
  // graph.Add("left", tg::RefFrame("camera"), left);
  // graph.Add("right", tg::RefFrame("camera"), right);
  // graph.Add("left rotated", tg::RefFrame("left"),
  //          tg::Transform(tg::Position(), z_axis));
  // graph.Add("right rotated", tg::RefFrame("right"),
  //          tg::Transform(tg::Position(), z_axis));

  // left_marker.id = 2;
  // left_marker.color.r = 0;
  // left_marker.color.b = 1;
  // tg::Transform rotated;
  // graph.ComputeDescription(tg::LocalFrame("left rotated"),
  //                         tg::RefFrame("camera"), &rotated);
  // rotated.ToPose(&left_marker.pose);
  // viz_.publish(left_marker);

  // right_marker.id = 2;
  // right_marker.color.r = 0;
  // right_marker.color.b = 1;
  // graph.ComputeDescription(tg::LocalFrame("right rotated"),
  //                         tg::RefFrame("camera"), &rotated);
  // rotated.ToPose(&right_marker.pose);
  // viz_.publish(right_marker);
}
}  // namespace pbi
