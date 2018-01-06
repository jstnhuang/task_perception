#include "task_perception/annotator_server.h"

#include <map>
#include <memory>

#include "absl/strings/str_split.h"
#include "boost/shared_ptr.hpp"
//#include "dbot/object_resource_identifier.h"
//#include "dbot_ros/util/interactive_marker_initializer.h"
#include "geometry_msgs/Transform.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "skin_segmentation_msgs/AdvanceSkeleton.h"
#include "skin_segmentation_msgs/NerfJointStates.h"
#include "skin_segmentation_msgs/ResetSkeletonTracker.h"
#include "task_perception_msgs/AnnotatorEvent.h"
#include "task_perception_msgs/AnnotatorState.h"

#include "task_perception/bag_utils.h"
#include "task_perception/database.h"
#include "task_perception/demo_model.h"
#include "task_perception/names.h"
//#include "task_perception/particle_tracker_builder.h"
//#include "task_perception/track.h"
#include "task_perception/video_scrubber.h"

namespace msgs = task_perception_msgs;
namespace ss_msgs = skin_segmentation_msgs;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;
typedef message_filters::sync_policies::ExactTime<Image, Image> MyPolicy;
// typedef dbot::ObjectTrackerRos<dbot::ParticleTracker> ParticleTrackerRos;
// typedef std::shared_ptr<ParticleTrackerRos> ParticleTrackerRosPtr;

namespace pbi {
AnnotatorServer::AnnotatorServer(const ros::Publisher& camera_info_pub,
                                 const ros::Publisher& color_pub,
                                 const ros::Publisher& depth_pub,
                                 const ros::Publisher& state_pub,
                                 const ros::Publisher& nerf_pub,
                                 const DemonstrationDb& demo_db)
    : camera_info_pub_(camera_info_pub),
      color_pub_(color_pub),
      depth_pub_(depth_pub),
      state_pub_(state_pub),
      nerf_pub_(nerf_pub),
      demo_db_(demo_db),
      nh_(),
      timer_(nh_.createTimer(ros::Duration(1 / 15.0), &AnnotatorServer::Loop,
                             this)),
      bag_(),
      color_topic_(""),
      depth_topic_(""),
      camera_info_(),
      demo_id_(""),
      demo_model_(),
      state_(),
      color_scrubber_(),
      depth_scrubber_(),
      current_color_image_(),
      current_depth_image_(),
      reset_skeleton(nh_.serviceClient<ss_msgs::ResetSkeletonTracker>(
          "reset_skeleton_tracker")),
      advance_skeleton(
          nh_.serviceClient<ss_msgs::AdvanceSkeleton>("advance_skeleton")) {
  while (!reset_skeleton.waitForExistence(ros::Duration(2.0)) ||
         !advance_skeleton.waitForExistence(ros::Duration(2.0))) {
    ROS_WARN("Waiting for skeleton tracking service");
  }
}

void AnnotatorServer::Start() { state_pub_.publish(state_); }

void AnnotatorServer::HandleEvent(
    const task_perception_msgs::AnnotatorEvent& event) {
  try {
    if (event.type == msgs::AnnotatorEvent::OPEN_BAG) {
      HandleOpen(event.bag_path);
    } else if (event.type == msgs::AnnotatorEvent::STEP) {
      HandleStep();
    } else if (event.type == msgs::AnnotatorEvent::ADD_OBJECT) {
      HandleAddObject(event.mesh_name);
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

  GetImageTopics(*bag_, &color_topic_, &depth_topic_);
  GetCameraInfo(*bag_, &camera_info_);
  int num_frames = GetNumMessagesOnTopic(*bag_, depth_topic_);

  // Load images
  std::vector<std::string> topics(2);
  topics[0] = color_topic_;
  topics[1] = depth_topic_;
  rosbag::View view(*bag_, rosbag::TopicQuery(topics));
  std::vector<Image> color_images;
  std::vector<Image> depth_images;
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    if (it->getTopic() == color_topic_) {
      color_images.push_back(*it->instantiate<Image>());
    } else if (it->getTopic() == depth_topic_) {
      depth_images.push_back(*it->instantiate<Image>());
    }
  }
  color_scrubber_.set_images(color_images);
  depth_scrubber_.set_images(depth_images);

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
  demo_model_.reset(new DemoModel(demo));

  // Reset skeleton tracker
  ss_msgs::ResetSkeletonTrackerRequest reset_req;
  ss_msgs::ResetSkeletonTrackerResponse reset_res;
  reset_req.rgb_topic = color_topic_;
  reset_req.depth_topic = depth_topic_;
  reset_req.camera_info = camera_info_;
  reset_skeleton.call(reset_req, reset_res);

  // Initial pose for the skeleton
  if (!demo_model_->HasEventAt(msgs::Event::SET_SKELETON_STATE, 0)) {
    msgs::Event event;
    event.type = msgs::Event::SET_SKELETON_STATE;
    event.nerf_joint_states = DefaultSkeleton();
    demo_model_->AddEvent(event);
    demo_db_.Update(demo_id_, demo_model_->ToMsg());
  }

  ROS_INFO("Opened bag: %s with %d frames", bag_path.c_str(), num_frames);
  state_.bag_path = bag_path;
  state_.frame_count = num_frames;
  state_.current_frame = 0;

  ProcessCurrentStep();
}

void AnnotatorServer::HandleStep() {
  state_.current_frame += 1;
  if (state_.current_frame >= state_.frame_count) {
    ROS_INFO("Reached end of bag file.");
    return;
  }
  ProcessCurrentStep();
}

void AnnotatorServer::HandleAddObject(const std::string& mesh_name) {
  // TODO: we assume object exists since the beginning of the demonstration
  // i.e., depth_frame is 0.
  // if (tracks_.find(mesh_name) == tracks_.end()) {
  //  dbot::ObjectResourceIdentifier ori;
  //  BuildOri(nh_, mesh_name, &ori);
  //  Track track(nh_, ori, &depth_scrubber_);
  //  tracks_.insert(std::pair<std::string, Track>(mesh_name, track));

  //  ROS_INFO("Created tracker for %s", mesh_name.c_str());
  //}
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

  color_scrubber_.View(state_.current_frame, &current_color_image_);
  depth_scrubber_.View(state_.current_frame, &current_depth_image_);

  msgs::Event skel_event;
  if (demo_model_->EventAt(msgs::Event::SET_SKELETON_STATE,
                           state_.current_frame, &skel_event)) {
    nerf_pub_.publish(skel_event.nerf_joint_states);
  } else {
    ss_msgs::AdvanceSkeletonRequest advance_req;
    ss_msgs::AdvanceSkeletonResponse advance_res;
    advance_req.rgb = current_color_image_;
    advance_req.depth = current_depth_image_;
    advance_skeleton.call(advance_req, advance_res);
  }

  PublishState();
}

void AnnotatorServer::Loop(const ros::TimerEvent& event) {
  ros::Time now = ros::Time::now();
  CameraInfo cam_info = camera_info_;
  cam_info.header.stamp = now;
  camera_info_pub_.publish(cam_info);

  if (!current_color_image_.header.stamp.isZero()) {
    current_color_image_.header.stamp = now;
    color_pub_.publish(current_color_image_);
  }
  if (!current_depth_image_.header.stamp.isZero()) {
    current_depth_image_.header.stamp = now;
    depth_pub_.publish(current_depth_image_);
  }
}

void AnnotatorServer::PublishState() {
  state_pub_.publish(state_);

  // color_scrubber_.View(state_.current_frame, &current_color_image_);
  // depth_scrubber_.View(state_.current_frame, &current_depth_image_);

  // TODO: skeleton tracker manages its own state and visualization for now,
  // maybe move into here
}

// Hard coded joint states for an initial skeleton pose. The usual defaults are
// designed for a camera that is facing forward at person height. However, in
// our setup, the camera is usually looking down 45 degrees with the robot's
// torso all the way up. This lines up with a person standing about 2m  away
// from the robot.
ss_msgs::NerfJointStates DefaultSkeleton() {
  // Only the values field is used for updating.
  skin_segmentation_msgs::NerfJointStates js;
  js.values = {-0.1612870991230011,
               0.04867665097117424,
               2.3786327838897705,
               -0.05447574332356453,
               -0.05467025563120842,
               3.772512435913086,
               -0.029690731316804886,
               0.004630787763744593,
               -0.005021668039262295,
               0.00024390089674852788,
               -0.003092646598815918,
               0.006138387601822615,
               -0.011972501873970032,
               -0.11014781892299652,
               -0.02609388530254364,
               0.05859832465648651,
               0.01391539629548788,
               -0.04170745611190796,
               0.0021036609541624784,
               0.0005725773517042398,
               0.006801145151257515,
               -0.0026443470269441605,
               0.022352147847414017,
               -0.015505659393966198,
               0.025999244302511215,
               0.14551089704036713,
               -0.2196994572877884,
               0.07975294440984726,
               -0.019698331132531166,
               -0.304276704788208,
               0.06515953689813614,
               -0.01693911664187908,
               -0.22153688967227936,
               0.1447325199842453,
               0.05018325522542,
               0.04819465056061745,
               -0.034020453691482544,
               0.01699198968708515,
               0.007856975309550762,
               -0.03827618062496185,
               0.12328708916902542,
               0.03772320970892906,
               0.0014631988015025854,
               -0.05484601855278015,
               0.014221321791410446,
               -0.0809585377573967,
               -0.006737143732607365,
               0.15774470567703247,
               -0.2952468693256378,
               -0.18427912890911102,
               -0.01636110059916973,
               0.18570122122764587,
               -0.4172581434249878,
               -0.16075001657009125,
               -0.013778984546661377,
               0.029999999329447746,
               -0.0056851026602089405,
               0.013106471858918667,
               -0.013814818114042282,
               0.07387726753950119,
               0.002827439457178116,
               0.10885165631771088,
               0.019999999552965164,
               -0.13909637928009033,
               0.0708390474319458,
               0.019999999552965164,
               -0.06438956409692764,
               -0.14158183336257935,
               0.1297045350074768,
               0.04735807329416275,
               -0.009104527533054352,
               0.005478684324771166,
               -0.005285603925585747,
               0.011366375721991062,
               -0.06579946726560593,
               -0.023101815953850746,
               -0.0030594400595873594,
               0.03586011379957199,
               -0.13240891695022583,
               -0.03403475135564804,
               -0.0007981206872500479,
               0.07711388915777206,
               -0.142836794257164,
               -0.0040811048820614815,
               -0.0007981206872500479,
               0.02646607905626297,
               -0.08380138874053955,
               -7.616788207087666e-05,
               -0.0007981206872500479,
               -0.0071240924298763275,
               -0.013288114219903946,
               -0.00411814684048295,
               -0.08095583319664001,
               0.02189975045621395,
               -0.012012498453259468,
               -0.01901327632367611,
               0.03627721220254898,
               0.004438650328665972,
               0.0007578099030070007,
               0.001045851269736886,
               0.0,
               0.005632625427097082,
               0.019999999552965164,
               -0.018569445237517357,
               0.011239204555749893,
               0.04150545224547386,
               0.002742839977145195,
               0.028484275564551353,
               0.00017102464335039258,
               -0.004937081132084131,
               0.012813234701752663,
               0.001045851269736886,
               0.0};
  return js;
}
}  // namespace pbi
