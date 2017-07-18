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

using dbot::ParticleTracker;
typedef dbot::ObjectTrackerRos<ParticleTracker> ParticleTrackerRos;

namespace pbi {
ParticleTrackerBuilder::ParticleTrackerBuilder(const ros::NodeHandle& nh)
    : nh_(nh), ori_() {}

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
  // get object parameters
  std::string object_package;
  std::string object_directory;

  /// \todo nh_.getParam does not check whether the parameter exists in the
  /// config file. this is dangerous, we should use ri::read instead
  nh_.getParam("object/package", object_package);
  nh_.getParam("object/directory", object_directory);

  // Use the ORI to load the object model usign the
  // SimpleWavefrontObjectLoader
  if (ori_.count_meshes() == 0) {
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
  std::string camera_info_topic;
  std::string depth_image_topic;
  dbot::CameraData::Resolution resolution;
  nh_.getParam("camera_info_topic", camera_info_topic);
  nh_.getParam("depth_image_topic", depth_image_topic);
  nh_.getParam("downsampling_factor", downsampling_factor);
  nh_.getParam("resolution/width", resolution.width);
  nh_.getParam("resolution/height", resolution.height);

  auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
      new dbot::RosCameraDataProvider(nh_, camera_info_topic, depth_image_topic,
                                      resolution, downsampling_factor, 60.0));
  // Create camera data from the RosCameraDataProvider which takes the data
  // from a ros camera topic
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

  int downsampling_factor;
  std::string camera_info_topic;
  std::string depth_image_topic;
  dbot::CameraData::Resolution resolution;
  nh_.getParam("camera_info_topic", camera_info_topic);
  nh_.getParam("depth_image_topic", depth_image_topic);
  nh_.getParam("downsampling_factor", downsampling_factor);
  nh_.getParam("resolution/width", resolution.width);
  nh_.getParam("resolution/height", resolution.height);

  auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
      new dbot::RosCameraDataProvider(nh_, camera_info_topic, depth_image_topic,
                                      resolution, downsampling_factor, 60.0));
  // Create camera data from the RosCameraDataProvider which takes the data
  // from a ros camera topic
  auto camera_data = std::make_shared<dbot::CameraData>(camera_data_provider);

  std::shared_ptr<ParticleTrackerRos> ros_tracker(
      new ParticleTrackerRos(tracker, camera_data, ori_.count_meshes()));
  return ros_tracker;
}
}  // namespace pbi
