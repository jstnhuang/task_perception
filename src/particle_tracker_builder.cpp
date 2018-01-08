#include "task_perception/particle_tracker_builder.h"

#include <memory>
#include <string>
#include <vector>

#include "dbot/builder/particle_tracker_builder.h"
#include "dbot/camera_data.h"
#include "dbot/simple_wavefront_object_loader.h"
#include "dbot/tracker/particle_tracker.h"
#include "dbot_ros/util/ros_camera_data_provider.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

#include "task_perception/camera_info_camera_provider.h"

using dbot::ParticleTracker;
typedef dbot::ObjectTrackerRos<ParticleTracker> ParticleTrackerRos;

namespace pbi {
ParticleTrackerBuilder::ParticleTrackerBuilder(
    const ros::NodeHandle& nh, const sensor_msgs::CameraInfo& camera_info)
    : nh_(nh), camera_info_(camera_info), ori_() {}

void ParticleTrackerBuilder::set_object(
    const dbot::ObjectResourceIdentifier& ori) {
  ori_ = ori;
}

std::shared_ptr<dbot::ParticleTracker> ParticleTrackerBuilder::Build() {
  // parameter shorthand prefix
  std::string pre = "particle_filter/";

  /* ------------------------------ */
  /* - Create the object model    - */
  /* ------------------------------ */
  // Use the ORI to load the object model usign the
  // SimpleWavefrontObjectLoader
  if (ori_.count_meshes() == 0) {
    ROS_ERROR("No meshes given in ORI!");
    return nullptr;
  }

  auto object_model_loader = std::shared_ptr<dbot::ObjectModelLoader>(
      new dbot::SimpleWavefrontObjectModelLoader(ori_));

  // Load the model usign the simple wavefront load and center the frames
  // of all object part meshes
  bool center_object_frame;
  nh_.getParam(pre + "center_object_frame", center_object_frame);
  auto object_model = std::make_shared<dbot::ObjectModel>(object_model_loader,
                                                          center_object_frame);

  /* ------------------------------ */
  /* - Setup camera data          - */
  /* ------------------------------ */
  int downsampling_factor;
  nh_.getParam("downsampling_factor", downsampling_factor);

  auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
      new CameraInfoCameraProvider(camera_info_, downsampling_factor));
  auto camera_data = std::make_shared<dbot::CameraData>(camera_data_provider);

  /* ------------------------------ */
  /* - Few types we will be using - */
  /* ------------------------------ */
  typedef dbot::FreeFloatingRigidBodiesState<> State;
  typedef dbot::ParticleTracker Tracker;
  typedef dbot::ParticleTrackerBuilder<Tracker> TrackerBuilder;
  typedef TrackerBuilder::TransitionBuilder TransitionBuilder;
  typedef TrackerBuilder::SensorBuilder SensorBuilder;

  /* ------------------------------ */
  /* - State transition function  - */
  /* ------------------------------ */
  // We will use a linear observation model built by the object transition
  // model builder. The linear model will generate a random walk.
  dbot::ObjectTransitionBuilder<State>::Parameters params_state;
  // state transition parameters
  nh_.getParam(pre + "object_transition/linear_sigma_x",
               params_state.linear_sigma_x);
  nh_.getParam(pre + "object_transition/linear_sigma_y",
               params_state.linear_sigma_y);
  nh_.getParam(pre + "object_transition/linear_sigma_z",
               params_state.linear_sigma_z);

  nh_.getParam(pre + "object_transition/angular_sigma_x",
               params_state.angular_sigma_x);
  nh_.getParam(pre + "object_transition/angular_sigma_y",
               params_state.angular_sigma_y);
  nh_.getParam(pre + "object_transition/angular_sigma_z",
               params_state.angular_sigma_z);

  nh_.getParam(pre + "object_transition/velocity_factor",
               params_state.velocity_factor);
  params_state.part_count = ori_.count_meshes();

  auto state_trans_builder = std::shared_ptr<TransitionBuilder>(
      new dbot::ObjectTransitionBuilder<State>(params_state));

  /* ------------------------------ */
  /* - Observation model          - */
  /* ------------------------------ */
  dbot::RbSensorBuilder<State>::Parameters params_obsrv;
  nh_.getParam(pre + "use_gpu", params_obsrv.use_gpu);

  if (params_obsrv.use_gpu) {
    nh_.getParam(pre + "gpu/sample_count", params_obsrv.sample_count);
  } else {
    nh_.getParam(pre + "cpu/sample_count", params_obsrv.sample_count);
  }

  nh_.getParam(pre + "observation/occlusion/p_occluded_visible",
               params_obsrv.occlusion.p_occluded_visible);
  nh_.getParam(pre + "observation/occlusion/p_occluded_occluded",
               params_obsrv.occlusion.p_occluded_occluded);
  nh_.getParam(pre + "observation/occlusion/initial_occlusion_prob",
               params_obsrv.occlusion.initial_occlusion_prob);

  nh_.getParam(pre + "observation/kinect/tail_weight",
               params_obsrv.kinect.tail_weight);
  nh_.getParam(pre + "observation/kinect/model_sigma",
               params_obsrv.kinect.model_sigma);
  nh_.getParam(pre + "observation/kinect/sigma_factor",
               params_obsrv.kinect.sigma_factor);
  params_obsrv.delta_time = 1. / 30.;

  // gpu only parameters
  nh_.getParam(pre + "gpu/use_custom_shaders", params_obsrv.use_custom_shaders);
  nh_.getParam(pre + "gpu/vertex_shader_file", params_obsrv.vertex_shader_file);
  nh_.getParam(pre + "gpu/fragment_shader_file",
               params_obsrv.fragment_shader_file);
  nh_.getParam(pre + "gpu/geometry_shader_file",
               params_obsrv.geometry_shader_file);

  auto sensor_builder =
      std::shared_ptr<SensorBuilder>(new dbot::RbSensorBuilder<State>(
          object_model, camera_data, params_obsrv));

  /* ------------------------------ */
  /* - Create Filter & Tracker    - */
  /* ------------------------------ */
  TrackerBuilder::Parameters params_tracker;
  params_tracker.evaluation_count = params_obsrv.sample_count;
  nh_.getParam(pre + "moving_average_update_rate",
               params_tracker.moving_average_update_rate);
  nh_.getParam(pre + "max_kl_divergence", params_tracker.max_kl_divergence);
  nh_.getParam(pre + "center_object_frame", params_tracker.center_object_frame);

  auto tracker_builder = dbot::ParticleTrackerBuilder<Tracker>(
      state_trans_builder, sensor_builder, object_model, params_tracker);
  return tracker_builder.build();
}

std::shared_ptr<ParticleTrackerRos> ParticleTrackerBuilder::BuildRos() {
  auto tracker = Build();
  auto camera_data = BuildCameraData(nh_);
  std::shared_ptr<ParticleTrackerRos> ros_tracker(
      new ParticleTrackerRos(tracker, camera_data, ori_.count_meshes()));
  return ros_tracker;
}

void BuildOri(const ros::NodeHandle& nh, const std::string& mesh_name,
              dbot::ObjectResourceIdentifier* ori) {
  std::string object_package;
  std::string object_directory;
  nh.getParam("object/package", object_package);
  nh.getParam("object/directory", object_directory);
  ori->package_path(ros::package::getPath(object_package));
  ori->directory(object_directory);
  ori->meshes({mesh_name});
}

std::shared_ptr<dbot::CameraData> BuildCameraData(const ros::NodeHandle& nh) {
  int downsampling_factor;
  std::string camera_info_topic;
  std::string depth_image_topic;
  dbot::CameraData::Resolution resolution;
  nh.getParam("camera_info_topic", camera_info_topic);
  nh.getParam("depth_image_topic", depth_image_topic);
  nh.getParam("downsampling_factor", downsampling_factor);
  nh.getParam("resolution/width", resolution.width);
  nh.getParam("resolution/height", resolution.height);
  auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
      new dbot::RosCameraDataProvider(nh, camera_info_topic, depth_image_topic,
                                      resolution, downsampling_factor, 60.0));
  // Create camera data from the RosCameraDataProvider which takes the data
  // from a ros camera topic
  return std::make_shared<dbot::CameraData>(camera_data_provider);
}
}  // namespace pbi
