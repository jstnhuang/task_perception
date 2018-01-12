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
      demo_runtime_(demo_viz_, skel_services_),
      state_() {}

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
      HandleDeleteEvent(event.event_type, event.object_name);
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

  ROS_INFO("Opened bag: %s with %d frames", bag_path.c_str(), num_frames);
  state_.bag_path = bag_path;
  state_.frame_count = num_frames;
  state_.current_frame = 0;
  ProcessCurrentStep();
}

void AnnotatorServer::HandleStep() {
  if (!bag_) {
    ROS_ERROR("No bag file loaded");
    return;
  }

  state_.current_frame += 1;
  if (state_.current_frame >= state_.frame_count) {
    ROS_INFO("Reached end of bag file.");
    return;
  }
  ProcessCurrentStep();
}

void AnnotatorServer::HandleSaveSkeleton() {
  if (!bag_) {
    ROS_ERROR("No bag file loaded");
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
  ProcessCurrentStep();
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

void AnnotatorServer::HandleDeleteEvent(const std::string& event_type,
                                        const std::string& object_name) {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  msgs::Event event;
  event.type = event_type;
  event.object_name = object_name;
  demo_model_->DeleteEvent(event, state_.current_frame);
  demo_db_.Update(demo_id_, demo_model_->ToMsg());
  PublishState();
}

void AnnotatorServer::HandleAddObject(const std::string& object_name,
                                      const std::string& mesh_name) {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  msgs::Event event;
  event.frame_number = state_.current_frame;
  event.type = msgs::Event::SPAWN_OBJECT;
  event.object_name = object_name;
  event.object_mesh = mesh_name;
  demo_model_->AddEvent(event);
  demo_db_.Update(demo_id_, demo_model_->ToMsg());
  PublishState();
}

void AnnotatorServer::HandleRemoveObject(const std::string& object_name) {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  msgs::Event event;
  event.frame_number = state_.current_frame;
  event.type = msgs::Event::UNSPAWN_OBJECT;
  event.object_name = object_name;
  demo_model_->AddEvent(event);
  demo_db_.Update(demo_id_, demo_model_->ToMsg());
  PublishState();
}

void AnnotatorServer::ProcessCurrentStep() {
  if (!demo_model_) {
    ROS_ERROR("No demo model loaded");
    return;
  }
  if (state_.current_frame < 0 || state_.current_frame >= state_.frame_count) {
    ROS_ERROR("Invalid step %d (%d).", state_.current_frame,
              state_.frame_count);
    return;
  }

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
  state_.events = demo_model_->EventsAt(state_.current_frame);
  demo_viz_.state_pub.publish(state_);

  // TODO: skeleton tracker manages its own state and visualization for now,
  // maybe move into here
}

}  // namespace pbi
