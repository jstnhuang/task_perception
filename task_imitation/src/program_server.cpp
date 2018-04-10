#include "task_imitation/program_server.h"

#include <limits.h>
#include <map>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_ros/params.h"
#include "rapid_utils/pcl_typedefs.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "surface_perception/segmentation.h"
#include "surface_perception/visualization.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception_msgs/DemoStates.h"
#include "task_perception_msgs/GenerateProgramAction.h"
#include "task_perception_msgs/GetDemoStates.h"
#include "task_perception_msgs/ImitateDemoAction.h"
#include "task_perception_msgs/ImitationEvent.h"
#include "task_perception_msgs/Program.h"
#include "task_utils/bag_utils.h"
#include "transform_graph/graph.h"
#include "visualization_msgs/Marker.h"

#include "task_imitation/program_executor.h"
#include "task_imitation/program_generator.h"

namespace msgs = task_perception_msgs;
namespace tg = transform_graph;
using boost::optional;
using geometry_msgs::Pose;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace pbi {
ProgramServer::ProgramServer(
    const ros::ServiceClient& db_client,
    const rapid::PointCloudCameraInterface& cam_interface)
    : db_client_(db_client),
      cam_interface_(cam_interface),
      nh_(),
      generate_program_server_(
          nh_, "generate_program",
          boost::bind(&pbi::ProgramServer::GenerateProgram, this, _1), false),
      imitate_demo_server_(
          nh_, "imitate_demo",
          boost::bind(&pbi::ProgramServer::ExecuteImitation, this, _1), false),
      initialize_object_("initialize_object"),
      left_group_("left_arm"),
      right_group_("right_arm"),
      executor_(left_group_, right_group_),
      program_pub_(
          nh_.advertise<msgs::Program>("pbi_imitation/program", 1, true)),
      cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>(
          "pbi_imitation/point_cloud", 1, true)),
      segmentation_pub_(nh_.advertise<visualization_msgs::Marker>(
          "pbi_imitation/surface_segmentation", 10)),
      segmentation_viz_(segmentation_pub_),
      model_cache_(),
      planning_frame_(left_group_.getPlanningFrame()),
      program_(),
      object_states_(),
      marker_arr_(),
      kGripperMarkers(),
      marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "pbi_imitation/markers", 100)) {}

void ProgramServer::Start() {
  urdf::Model model;
  model.initParam("robot_description");
  robot_markers::Builder builder(model);
  builder.Init();
  std::map<std::string, double> joint_positions;
  joint_positions["l_gripper_joint"] = 0.088;
  joint_positions["l_gripper_l_finger_joint"] = 0.514;
  joint_positions["l_gripper_l_finger_tip_joint"] = 0.514;
  joint_positions["l_gripper_r_finger_joint"] = 0.514;
  joint_positions["l_gripper_r_finger_tip_joint"] = 0.514;
  builder.SetJointPositions(joint_positions);

  std::set<std::string> gripper_links;
  gripper_links.insert("l_gripper_palm_link");
  gripper_links.insert("l_gripper_l_finger_link");
  gripper_links.insert("l_gripper_l_finger_tip_link");
  gripper_links.insert("l_gripper_r_finger_link");
  gripper_links.insert("l_gripper_r_finger_tip_link");

  builder.Build(gripper_links, &kGripperMarkers);

  // Shift palm to origin / identity orientation.
  geometry_msgs::Pose root_pose;
  for (size_t i = 0; i < kGripperMarkers.markers.size(); ++i) {
    if (kGripperMarkers.markers[i].mesh_resource.find("palm") !=
        std::string::npos) {
      root_pose = kGripperMarkers.markers[i].pose;
      break;
    }
  }
  Eigen::Affine3d gripper_pose;
  tf::poseMsgToEigen(root_pose, gripper_pose);

  for (size_t i = 0; i < kGripperMarkers.markers.size(); ++i) {
    visualization_msgs::Marker& marker = kGripperMarkers.markers[i];
    Eigen::Affine3d marker_pose;
    tf::poseMsgToEigen(marker.pose, marker_pose);
    Eigen::Affine3d shifted_pose = gripper_pose.inverse() * marker_pose;
    tf::poseEigenToMsg(shifted_pose, marker.pose);
  }

  generate_program_server_.start();
  imitate_demo_server_.start();
  while (ros::ok() && !initialize_object_.waitForServer(ros::Duration(2.0))) {
    ROS_WARN("Waiting for object initializer action.");
  }
  executor_.Init();
}

void ProgramServer::GenerateProgram(
    const msgs::GenerateProgramGoalConstPtr& goal) {
  std::string error =
      GenerateProgramInternal(goal->bag_path, &program_, &object_states_);
  if (error != "") {
    ROS_ERROR("%s", error.c_str());
    msgs::GenerateProgramResult result;
    result.error = error;
    generate_program_server_.setAborted(result, error);
    return;
  }

  msgs::GenerateProgramResult result;
  generate_program_server_.setSucceeded(result);
  segmentation_viz_.Hide();
}

void ProgramServer::ExecuteImitation(
    const msgs::ImitateDemoGoalConstPtr& goal) {
  std::string error =
      GenerateProgramInternal(goal->bag_path, &program_, &object_states_);
  if (error != "") {
    ROS_ERROR("%s", error.c_str());
    msgs::ImitateDemoResult result;
    result.error = error;
    imitate_demo_server_.setAborted(result, error);
    return;
  }

  executor_.Execute(program_, object_states_);
  msgs::ImitateDemoResult result;
  imitate_demo_server_.setSucceeded(result);
  segmentation_viz_.Hide();
}

void ProgramServer::HandleEvent(const msgs::ImitationEvent& event) {
  if (event.type == msgs::ImitationEvent::VISUALIZE) {
    VisualizeStep(event.step);
  } else {
    ROS_ERROR("Unknown imitation event: \"%s\"", event.type.c_str());
  }
}

std::string ProgramServer::GenerateProgramInternal(
    const std::string& bag_path, msgs::Program* program,
    std::map<std::string, msgs::ObjectState>* object_states) {
  msgs::GetDemoStatesRequest get_states_req;
  get_states_req.name = GetNameFromBagPath(bag_path);
  msgs::GetDemoStatesResponse get_states_res;
  db_client_.call(get_states_req, get_states_res);
  if (get_states_res.error != "") {
    return get_states_res.error;
  }

  // Generate program and slices
  const msgs::DemoStates& demo_states = get_states_res.demo_states;
  *object_states = GetObjectPoses(demo_states);
  ROS_INFO("All object states initialized.");

  ProgramGenerator generator(left_group_, right_group_);
  *program = generator.Generate(demo_states.demo_states, *object_states);
  program_pub_.publish(*program);
  return "";
}

std::map<std::string, msgs::ObjectState> ProgramServer::GetObjectPoses(
    const msgs::DemoStates& demo_states) {
  // Get point cloud in base frame
  rapid::PointCloudC::Ptr cloud = cam_interface_.cloud();
  geometry_msgs::TransformStamped cam_in_base = cam_interface_.camera_pose();
  Eigen::Affine3d transform;
  tf::transformMsgToEigen(cam_in_base.transform, transform);
  rapid::PointCloudC::Ptr cloud_in_base(new rapid::PointCloudC);
  pcl::transformPointCloud(*cloud, *cloud_in_base, transform);
  cloud_in_base->header.frame_id = cam_in_base.header.frame_id;

  pcl::CropBox<rapid::PointC> crop;
  crop.setInputCloud(cloud_in_base);
  double min_x =
      rapid::GetDoubleParamOrThrow("surface_segmentation/crop_min_x");
  double min_y =
      rapid::GetDoubleParamOrThrow("surface_segmentation/crop_min_y");
  double min_z =
      rapid::GetDoubleParamOrThrow("surface_segmentation/crop_min_z");
  double max_x =
      rapid::GetDoubleParamOrThrow("surface_segmentation/crop_max_x");
  double max_y =
      rapid::GetDoubleParamOrThrow("surface_segmentation/crop_max_y");
  double max_z =
      rapid::GetDoubleParamOrThrow("surface_segmentation/crop_max_z");
  Eigen::Vector4f min_pt;
  min_pt << min_x, min_y, min_z, 1;
  crop.setMin(min_pt);
  Eigen::Vector4f max_pt;
  max_pt << max_x, max_y, max_z, 1;
  crop.setMax(max_pt);
  rapid::PointCloudC::Ptr cropped_cloud(new rapid::PointCloudC);
  crop.filter(*cropped_cloud);

  sensor_msgs::PointCloud2 cloud_in_base_msg;
  pcl::toROSMsg(*cropped_cloud, cloud_in_base_msg);
  cloud_in_base_msg.header.frame_id = cam_in_base.header.frame_id;
  cloud_pub_.publish(cloud_in_base_msg);

  surface_perception::Segmentation seg;
  seg.set_input_cloud(cropped_cloud);
  seg.set_horizontal_tolerance_degrees(rapid::GetDoubleParamOrThrow(
      "surface_segmentation/horizontal_tolerance_degrees"));
  seg.set_margin_above_surface(rapid::GetDoubleParamOrThrow(
      "surface_segmentation/margin_above_surface"));
  seg.set_cluster_distance(
      rapid::GetDoubleParamOrThrow("surface_segmentation/cluster_distance"));
  seg.set_min_cluster_size(
      rapid::GetDoubleParamOrThrow("surface_segmentation/min_cluster_size"));
  seg.set_max_cluster_size(
      rapid::GetDoubleParamOrThrow("surface_segmentation/max_cluster_size"));
  seg.set_min_surface_size(
      rapid::GetDoubleParamOrThrow("surface_segmentation/min_surface_size"));
  std::vector<surface_perception::SurfaceObjects> surface_objects;
  bool segment_success = seg.Segment(&surface_objects);
  if (!segment_success) {
    ROS_ERROR("Failed to segment surface.");
  }
  segmentation_viz_.Hide();
  segmentation_viz_.set_surface_objects(surface_objects);
  segmentation_viz_.Show();

  // This models the assumption that each demonstration only interacts with an
  // object once.We could / should allow the robot to interact with an object
  // more than once, but we need to continuosly track the objects in that
  // case.
  std::map<std::string, msgs::ObjectState> object_states;
  for (size_t i = 0; i < demo_states.demo_states.size(); ++i) {
    const msgs::DemoState& state = demo_states.demo_states[i];
    for (size_t j = 0; j < state.object_states.size(); ++j) {
      const msgs::ObjectState& os = state.object_states[j];
      if (object_states.find(os.name) == object_states.end()) {
        object_states[os.name] = os;
      }
    }
  }

  // For each object in the demonstration, see if there's a close match in the
  // segmented objects. If so, initialize its position automatically. Otherwise,
  // bring up an interactive marker.
  for (std::map<std::string, msgs::ObjectState>::iterator it =
           object_states.begin();
       it != object_states.end(); ++it) {
    const msgs::ObjectState& obj_state = it->second;
    LazyObjectModel obj_model(obj_state.mesh_name, planning_frame_,
                              obj_state.pose);
    obj_model.set_object_model_cache(&model_cache_);
    geometry_msgs::Vector3 obj_scale = obj_model.scale();

    geometry_msgs::Pose obj_pose;
    geometry_msgs::Vector3 obj_scale_obs;
    bool found =
        MatchObject(obj_scale, surface_objects, &obj_pose, &obj_scale_obs);
    dbot_ros_msgs::InitializeObjectGoal init_goal;
    init_goal.frame_id = executor_.planning_frame();
    init_goal.mesh_name = it->second.mesh_name;
    if (found) {
      // surface_perception returns a pose in the center of the object
      // However, our object models have their frames defined in the bottom
      // center of the object. So we shift the pose by half the height.
      tg::Graph graph;
      graph.Add("center frame", tg::RefFrame("planning"), obj_pose);
      tg::Position origin_in_planning;
      graph.DescribePosition(tg::Position(0, 0, -obj_scale.z / 2),
                             tg::Source("center frame"), tg::Target("planning"),
                             &origin_in_planning);
      init_goal.initial_pose = obj_pose;
      init_goal.initial_pose.position = origin_in_planning.point();
    } else {
      init_goal.initial_pose.orientation.w = 1;
      init_goal.initial_pose.position.x = 1;
      init_goal.initial_pose.position.z = 1;
    }
    initialize_object_.sendGoal(init_goal);
    while (ros::ok() && !initialize_object_.getState().isDone()) {
      ros::spinOnce();
    }
    dbot_ros_msgs::InitializeObjectResultConstPtr init_result =
        initialize_object_.getResult();
    it->second.pose = init_result->pose;
    ROS_INFO_STREAM("Initialized pose for object: \""
                    << it->first << "\" at pose " << it->second.pose);
  }

  return object_states;
}

void ProgramServer::VisualizeStep(const msgs::Step& step) {
  // Clear previous visualization
  visualization_msgs::MarkerArray deleters;
  for (size_t i = 0; i < marker_arr_.markers.size(); ++i) {
    visualization_msgs::Marker marker = marker_arr_.markers[i];
    marker.action = visualization_msgs::Marker::DELETE;
    deleters.markers.push_back(marker);
  }
  marker_pub_.publish(deleters);
  marker_arr_.markers.clear();

  if (step.type == msgs::Step::GRASP) {
    msgs::ObjectState obj = object_states_[step.object_state.name];
    tg::Graph graph;
    graph.Add("obj", tg::RefFrame("planning"), obj.pose);
    graph.Add("grasp", tg::RefFrame("obj"), step.ee_trajectory[0]);
    graph.Add("pregrasp", tg::RefFrame("grasp"),
              tg::Transform(tg::Position(-0.1, 0, 0), tg::Orientation()));

    tg::Transform pregrasp_in_planning;
    graph.ComputeDescription("pregrasp", tg::RefFrame("planning"),
                             &pregrasp_in_planning);
    MarkerArray pregrasp_markers = GripperMarkers(
        "pregrasp", pregrasp_in_planning.pose(), planning_frame_);
    marker_arr_.markers.insert(marker_arr_.markers.end(),
                               pregrasp_markers.markers.begin(),
                               pregrasp_markers.markers.end());

    tg::Transform grasp_in_planning;
    graph.ComputeDescription("grasp", tg::RefFrame("planning"),
                             &grasp_in_planning);
    MarkerArray grasp_markers =
        GripperMarkers("grasp", grasp_in_planning.pose(), planning_frame_);
    marker_arr_.markers.insert(marker_arr_.markers.end(),
                               grasp_markers.markers.begin(),
                               grasp_markers.markers.end());
  } else if (step.type == msgs::Step::UNGRASP) {
  } else if (step.type == msgs::Step::FOLLOW_TRAJECTORY) {
  } else if (step.type == msgs::Step::MOVE_TO_POSE) {
    msgs::ObjectState obj = object_states_[step.object_state.name];
    tg::Graph graph;
    graph.Add("obj", tg::RefFrame("planning"), obj.pose);
    graph.Add("dest", tg::RefFrame("obj"), step.ee_trajectory[0]);

    tg::Transform dest_in_planning;
    graph.ComputeDescription("dest", tg::RefFrame("planning"),
                             &dest_in_planning);
    MarkerArray pregrasp_markers =
        GripperMarkers("dest", dest_in_planning.pose(), planning_frame_);
    marker_arr_.markers.insert(marker_arr_.markers.end(),
                               pregrasp_markers.markers.begin(),
                               pregrasp_markers.markers.end());
  }
  marker_pub_.publish(marker_arr_);
}

MarkerArray ProgramServer::GripperMarkers(const std::string& ns,
                                          const geometry_msgs::Pose& pose,
                                          const std::string& frame_id) {
  MarkerArray result;
  Eigen::Affine3d pose_transform;
  tf::poseMsgToEigen(pose, pose_transform);
  for (size_t i = 0; i < kGripperMarkers.markers.size(); ++i) {
    visualization_msgs::Marker marker = kGripperMarkers.markers[i];
    marker.header.frame_id = frame_id;
    marker.ns = ns;

    Eigen::Affine3d marker_pose;
    tf::poseMsgToEigen(marker.pose, marker_pose);
    Eigen::Affine3d shifted_pose = pose_transform * marker_pose;
    tf::poseEigenToMsg(shifted_pose, marker.pose);
    result.markers.push_back(marker);
  }
  return result;
}

bool MatchObject(
    const geometry_msgs::Vector3& obj_scale,
    const std::vector<surface_perception::SurfaceObjects>& surface_objects,
    geometry_msgs::Pose* pose, geometry_msgs::Vector3* scale) {
  double match_dim_tolerance;
  ros::param::param("match_dim_tolerance", match_dim_tolerance, 0.05);
  double best_sq_norm = std::numeric_limits<double>::max();
  for (size_t i = 0; i < surface_objects.size(); ++i) {
    const surface_perception::SurfaceObjects& surface = surface_objects[i];
    for (size_t j = 0; j < surface.objects.size(); ++j) {
      const surface_perception::Object& object = surface.objects[i];
      double dim_dx = fabs(object.dimensions.x - obj_scale.x);
      double dim_dy = fabs(object.dimensions.y - obj_scale.y);
      double dim_dz = fabs(object.dimensions.z - obj_scale.z);
      if (dim_dx < match_dim_tolerance && dim_dy < match_dim_tolerance &&
          dim_dz < match_dim_tolerance) {
        double sq_norm = dim_dx * dim_dx + dim_dy * dim_dy + dim_dz * dim_dz;
        if (sq_norm < best_sq_norm) {
          best_sq_norm = sq_norm;
          *pose = object.pose_stamped.pose;
          *scale = object.dimensions;
        }
      }
    }
  }
  return best_sq_norm != std::numeric_limits<double>::max();
}

}  // namespace pbi
