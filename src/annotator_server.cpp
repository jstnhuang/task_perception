#include "task_perception/annotator_server.h"

#include <memory>
#include <string>
#include <vector>

#include "absl/strings/str_split.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/AdvanceSkeleton.h"
#include "skin_segmentation_msgs/GetSkeletonState.h"
#include "skin_segmentation_msgs/NerfJointStates.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"

#include "task_perception/bag_utils.h"
#include "task_perception/database.h"
#include "task_perception/demo_model.h"
#include "task_perception/demo_runtime.h"
#include "task_perception/demo_visualizer.h"
#include "task_perception/names.h"
#include "task_perception/skeleton_services.h"

namespace msgs = task_perception_msgs;
namespace ss_msgs = skin_segmentation_msgs;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

namespace pbi {
AnnotatorServer::AnnotatorServer(const DemoVisualizer& demo_viz,
                                 const SkeletonServices& skel_services,
                                 const DemonstrationDb& demo_db)
    : demo_viz_(demo_viz),
      skel_services_(skel_services),
      demo_db_(demo_db),
      nh_(),
      bag_(),
      demo_id_(""),
      demo_model_(),
      state_(),
      demo_runtime_(demo_viz_, skel_services_),
      object_init_() {}

void AnnotatorServer::Start() { demo_viz_.state_pub.publish(state_); }

void AnnotatorServer::HandleEvent(
    const task_perception_msgs::AnnotatorEvent& event) {
  try {
    if (event.type == msgs::AnnotatorEvent::OPEN_BAG) {
      HandleOpen(event.bag_path);
    } else if (event.type == msgs::AnnotatorEvent::STEP) {
      HandleStep();
    } else if (event.type == msgs::AnnotatorEvent::ADD_OBJECT) {
      HandleAddObject(event.object_name, event.mesh_name);
    } else if (event.type == msgs::AnnotatorEvent::ADD_OBJECT) {
      HandleRemoveObject(event.object_name);
    } else if (event.type == msgs::AnnotatorEvent::SAVE_SKELETON) {
      HandleSaveSkeleton();
    } else if (event.type == msgs::AnnotatorEvent::STEP_SKELETON) {
      HandleAdvanceSkeleton();
    } else if (event.type == msgs::AnnotatorEvent::DELETE_EVENT) {
      HandleDeleteEvent(event);
    } else if (event.type == msgs::AnnotatorEvent::SET_OBJECT_POSE) {
      HandleSetObjectPose(event.object_name);
    } else {
      ROS_ERROR("Unknown event type: \"%s\"", event.type.c_str());
    }
  } catch (const rosbag::BagException& ex) {
    ROS_ERROR("rosbag exception: %s", ex.what());
    bag_.reset();
  }
}

void AnnotatorServer::HandleOpen(const std::string& bag_path) {
  if (bag_) {
    bag_->close();
  }
  bag_.reset(new rosbag::Bag);
  bag_->open(bag_path, rosbag::bagmode::Read);

  std::string color_topic;
  std::string depth_topic;
  GetImageTopics(*bag_, &color_topic, &depth_topic);
  GetCameraInfo(*bag_, &camera_info_);
  int num_frames = GetNumMessagesOnTopic(*bag_, depth_topic);

  // Load images
  std::vector<std::string> topics(2);
  topics[0] = color_topic;
  topics[1] = depth_topic;
  rosbag::View view(*bag_, rosbag::TopicQuery(topics));
  std::vector<Image> color_images;
  std::vector<Image> depth_images;
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    if (it->getTopic() == color_topic) {
      color_images.push_back(*it->instantiate<Image>());
    } else if (it->getTopic() == depth_topic) {
      depth_images.push_back(*it->instantiate<Image>());
    }
  }

  // Insert/Retrieve demonstration data from DB.
  std::vector<std::string> bag_path_parts = absl::StrSplit(bag_path, "/");
  std::string last_bag_part(bag_path_parts[bag_path_parts.size() - 1]);
  std::string bag_name(absl::StripSuffix(last_bag_part, ".bag"));

  demo_id_ = demo_db_.GetIdByName(bag_name);
  if (demo_id_ == "") {
    msgs::Demonstration demo_new;
    demo_new.name = bag_name;
    demo_new.frame_count = num_frames;
    demo_id_ = demo_db_.Insert(demo_new);
  }
  msgs::Demonstration demo;
  if (!demo_db_.Get(demo_id_, &demo)) {
    ROS_ERROR("Failed to get demonstration with ID: %s", demo_id_.c_str());
    return;
  }
  demo_db_.PublishDemonstration(demo_id_);
  demo_model_.reset(new DemoModel(demo));
  demo_runtime_.LoadDemo(color_topic, depth_topic, camera_info_, color_images,
                         depth_images, demo_model_);

  object_init_.reset(
      new opi::InteractiveMarkerInitializer(camera_info_.header.frame_id));

  ROS_INFO("Opened bag: %s with %d frames", bag_path.c_str(), num_frames);
  state_.bag_path = bag_path;
  state_.frame_count = num_frames;
  // current_frame points to the most recently executed frame, or -1 to start.
  state_.current_frame = -1;

  RunCurrentStep();
}

void AnnotatorServer::HandleStep() {
  if (!bag_) {
    ROS_ERROR("No bag file loaded");
    return;
  }

  int step_to_execute = state_.current_frame + 1;
  if (step_to_execute >= state_.frame_count) {
    ROS_INFO("Reached end of bag file.");
    return;
  }

  RunCurrentStep();
}

void AnnotatorServer::HandleSaveSkeleton() {
  if (!bag_) {
    ROS_ERROR("No bag file loaded");
    return;
  }
  if (state_.current_frame < 0 || state_.current_frame >= state_.frame_count) {
    ROS_ERROR("Invalid current frame %d (%d)", state_.current_frame,
              state_.frame_count);
    return;
  }

  msgs::Event event;
  event.frame_number = state_.current_frame;
  event.type = msgs::Event::SET_SKELETON_STATE;
  ss_msgs::GetSkeletonStateRequest get_req;
  ss_msgs::GetSkeletonStateResponse get_res;
  skel_services_.get_state.call(get_req, get_res);
  event.nerf_joint_states = get_res.nerf_joint_states;
  demo_model_->AddEvent(event);
  demo_db_.Update(demo_id_, demo_model_->ToMsg());

  RerunCurrentStep();
}

void AnnotatorServer::HandleAdvanceSkeleton() {
  if (!bag_) {
    ROS_ERROR("No bag file loaded");
    return;
  }
  sensor_msgs::Image color;
  sensor_msgs::Image depth;
  demo_runtime_.current_color_image(&color);
  demo_runtime_.current_depth_image(&depth);
  AdvanceSkeleton(color, depth);
}

void AnnotatorServer::HandleDeleteEvent(const msgs::AnnotatorEvent& event) {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  if (state_.current_frame < 0 || state_.current_frame >= state_.frame_count) {
    ROS_ERROR("Invalid current frame %d (%d)", state_.current_frame,
              state_.frame_count);
    return;
  }

  const std::string& event_type(event.event_type);
  if (event_type == msgs::Event::SPAWN_OBJECT) {
    demo_runtime_.RemoveSpawnObjectEvent(event.object_name);
  } else if (event_type == msgs::Event::UNSPAWN_OBJECT) {
    demo_runtime_.RemoveUnspawnObjectEvent(event.object_name, event.mesh_name);
  }

  msgs::Event evt;
  evt.type = event_type;
  evt.object_name = event.object_name;
  demo_model_->DeleteEvent(evt, state_.current_frame);
  demo_db_.Update(demo_id_, demo_model_->ToMsg());
  RerunCurrentStep();
}

void AnnotatorServer::HandleAddObject(const std::string& object_name,
                                      const std::string& mesh_name) {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  if (state_.current_frame < 0 || state_.current_frame >= state_.frame_count) {
    ROS_ERROR("Invalid current frame %d (%d)", state_.current_frame,
              state_.frame_count);
    return;
  }
  msgs::Event spawn_event;
  spawn_event.frame_number = state_.current_frame;
  spawn_event.type = msgs::Event::SPAWN_OBJECT;
  spawn_event.object_name = object_name;
  spawn_event.object_mesh = mesh_name;
  demo_model_->AddEvent(spawn_event);

  msgs::Event pose_event;
  pose_event.frame_number = state_.current_frame;
  pose_event.type = msgs::Event::SET_OBJECT_POSE;
  pose_event.object_name = object_name;
  SetObjectPose(object_name, mesh_name, &pose_event.object_pose);
  demo_model_->AddEvent(pose_event);

  demo_db_.Update(demo_id_, demo_model_->ToMsg());
  RerunCurrentStep();
}

void AnnotatorServer::HandleRemoveObject(const std::string& object_name) {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  if (state_.current_frame < 0 || state_.current_frame >= state_.frame_count) {
    ROS_ERROR("Invalid current frame %d (%d)", state_.current_frame,
              state_.frame_count);
    return;
  }
  msgs::Event event;
  event.frame_number = state_.current_frame;
  event.type = msgs::Event::UNSPAWN_OBJECT;
  event.object_name = object_name;
  demo_model_->AddEvent(event);
  demo_db_.Update(demo_id_, demo_model_->ToMsg());
  RerunCurrentStep();
}

void AnnotatorServer::HandleSetObjectPose(const std::string& object_name) {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  if (state_.current_frame < 0 || state_.current_frame >= state_.frame_count) {
    ROS_ERROR("Invalid current frame %d (%d)", state_.current_frame,
              state_.frame_count);
    return;
  }

  msgs::ObjectState object_state;
  bool found = demo_runtime_.GetObjectState(state_.current_frame, object_name,
                                            &object_state);
  if (!found) {
    ROS_ERROR("No object named \"%s\"", object_name.c_str());
    return;
  }

  msgs::Event event;
  event.frame_number = state_.current_frame;
  event.type = msgs::Event::SET_OBJECT_POSE;
  event.object_name = object_name;
  bool success =
      SetObjectPose(object_name, object_state.mesh_name, &event.object_pose);
  if (!success) {
    return;
  }

  demo_model_->AddEvent(event);
  demo_db_.Update(demo_id_, demo_model_->ToMsg());
  RerunCurrentStep();
}

bool AnnotatorServer::SetObjectPose(const std::string& object_name,
                                    const std::string& object_mesh,
                                    geometry_msgs::Pose* pose) {
  msgs::ObjectState object_state;
  demo_runtime_.GetObjectState(state_.current_frame, object_name,
                               &object_state);

  if (object_state.object_pose.position.x == 0 &&
      object_state.object_pose.position.y == 0 &&
      object_state.object_pose.position.z == 0 &&
      object_state.object_pose.orientation.w == 0 &&
      object_state.object_pose.orientation.x == 0 &&
      object_state.object_pose.orientation.y == 0 &&
      object_state.object_pose.orientation.z == 0) {
    object_state.object_pose.position.z = 1;
    object_state.object_pose.orientation.w = 1;
  }
  object_init_->set_object("object_meshes", "object_models", object_mesh,
                           object_state.object_pose, false);
  object_init_->wait_for_object_poses();
  if (object_init_->poses().size() == 0) {
    ROS_ERROR("Interactive marker not initialized properly!");
    return false;
  }
  *pose = object_init_->poses()[0];
  return true;
}

void AnnotatorServer::RunCurrentStep() {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  int step_to_execute = state_.current_frame + 1;
  if (step_to_execute < 0 || step_to_execute >= state_.frame_count) {
    ROS_ERROR("Invalid step %d (%d).", state_.current_frame,
              state_.frame_count);
    return;
  }
  state_.current_frame += 1;
  PublishState();
  demo_runtime_.Step();
  PublishState();
}

void AnnotatorServer::RerunCurrentStep() {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  if (state_.current_frame < 0 || state_.current_frame >= state_.frame_count) {
    ROS_ERROR("Invalid step %d (%d).", state_.current_frame,
              state_.frame_count);
    return;
  }
  demo_runtime_.RerunLastFrame();
  demo_runtime_.Step();
  PublishState();
}

void AnnotatorServer::AdvanceSkeleton(const sensor_msgs::Image& color,
                                      const sensor_msgs::Image& depth) {
  ss_msgs::AdvanceSkeletonRequest advance_req;
  ss_msgs::AdvanceSkeletonResponse advance_res;
  advance_req.rgb = color;
  advance_req.depth = depth;
  skel_services_.advance.call(advance_req, advance_res);
}

void AnnotatorServer::PublishState() {
  msgs::DemoState demo_state;
  demo_runtime_.GetState(state_.current_frame, &demo_state);
  state_.objects = demo_state.object_states;
  state_.events = demo_model_->EventsAt(state_.current_frame);
  // TODO: include objects here
  demo_viz_.state_pub.publish(state_);

  // TODO: skeleton tracker manages its own state and visualization for now,
  // maybe move into here
}

}  // namespace pbi
