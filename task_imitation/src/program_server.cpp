#include "task_imitation/program_server.h"

#include <map>
#include <string>
#include <vector>

#include "boost/optional.hpp"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "eigen_conversions/eigen_msg.h"
#include "moveit/robot_state/conversions.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rapid_utils/pcl_typedefs.h"
#include "task_perception/lazy_object_model.h"
#include "task_perception_msgs/ProgramSlice.h"
#include "task_utils/bag_utils.h"
#include "transform_graph/graph.h"
#include "visualization_msgs/Marker.h"

#include "task_imitation/object_initialization.h"
#include "task_imitation/program_constants.h"
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
      planning_scene_(),
      left_group_("left_arm"),
      right_group_("right_arm"),
      executor_(left_group_, right_group_),
      program_pub_(
          nh_.advertise<msgs::Program>("pbi_imitation/program", 1, true)),
      cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>(
          "pbi_imitation/point_cloud", 1, true)),
      segmentation_pub_(nh_.advertise<visualization_msgs::Marker>(
          "pbi_imitation/surface_segmentation", 100)),
      segmentation_viz_(segmentation_pub_),
      model_cache_(),
      planning_frame_(left_group_.getPlanningFrame()),
      program_(),
      object_states_(),
      marker_arr_(),
      marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>(
          "pbi_imitation/markers", 100)),
      traj_pub_(nh_.advertise<moveit_msgs::DisplayTrajectory>(
          "pbi_imitation/display_traj", 1)),
      gripper_viz_() {}

void ProgramServer::Start() {
  gripper_viz_.Init();

  generate_program_server_.start();
  imitate_demo_server_.start();
  while (ros::ok() && !initialize_object_.waitForServer(ros::Duration(2.0))) {
    ROS_WARN("Waiting for object initializer action.");
  }
  executor_.Init();
}

void ProgramServer::GenerateProgram(
    const msgs::GenerateProgramGoalConstPtr& goal) {
  object_states_.clear();
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
  object_states_.clear();
  std::string error =
      GenerateProgramInternal(goal->bag_path, &program_, &object_states_);
  if (error != "") {
    ROS_ERROR("%s", error.c_str());
    msgs::ImitateDemoResult result;
    result.error = error;
    imitate_demo_server_.setAborted(result, error);
    return;
  }

  error = executor_.Execute(program_, object_states_);
  if (error != "") {
    ROS_ERROR("%s", error.c_str());
    msgs::ImitateDemoResult result;
    result.error = error;
    imitate_demo_server_.setAborted(result, error);
    return;
  }
  msgs::ImitateDemoResult result;
  imitate_demo_server_.setSucceeded(result);
  segmentation_viz_.Hide();
}

void ProgramServer::HandleEvent(const msgs::ImitationEvent& event) {
  if (event.type == msgs::ImitationEvent::VISUALIZE) {
    VisualizeStep(event.step);
  } else if (event.type == msgs::ImitationEvent::VISUALIZE_SLICE) {
    VisualizeSlice(event.slice);
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

  planning_scene_.RemoveObstacle("table");

  // Generate program and slices
  const msgs::DemoStates& demo_states = get_states_res.demo_states;
  Obb table;
  GetObjectPoses(demo_states, object_states, &table);
  ROS_INFO("All object states initialized.");

  ProgramGenerator generator(left_group_, right_group_, &model_cache_,
                             gripper_viz_);
  *program = generator.Generate(demo_states.demo_states, *object_states, table);
  program_pub_.publish(*program);
  return "";
}

void ProgramServer::GetObjectPoses(
    const msgs::DemoStates& demo_states,
    std::map<std::string, msgs::ObjectState>* object_states, Obb* table) {
  rapid::PointCloudC::Ptr cloud_in_base = GetCloudInBase(cam_interface_);
  rapid::PointCloudC::Ptr cropped_cloud =
      CropCloudUsingParams(cloud_in_base, "surface_segmentation/crop_");
  sensor_msgs::PointCloud2 cloud_in_base_msg;
  pcl::toROSMsg(*cropped_cloud, cloud_in_base_msg);
  cloud_in_base_msg.header.frame_id =
      cam_interface_.camera_pose().header.frame_id;
  cloud_pub_.publish(cloud_in_base_msg);

  optional<std::vector<surface_perception::SurfaceObjects> > surface_objects =
      DetectTabletopObjects(cropped_cloud);
  if (!surface_objects) {
    ROS_ERROR("Failed to segment surface.");
  } else {
    if (surface_objects->size() > 0) {
      const surface_perception::SurfaceObjects& surface =
          surface_objects->at(0);
      table->pose = surface.surface.pose_stamped.pose;
      table->dims = surface.surface.dimensions;
      planning_scene_.AddBoxObstacle("table", surface.surface.pose_stamped,
                                     surface.surface.dimensions);
    }
    for (size_t i = 0; i < surface_objects->size(); ++i) {
      ROS_INFO("Surface %zu has %zu objects", i,
               surface_objects->at(i).objects.size());
    }

    segmentation_viz_.Hide();
    segmentation_viz_.set_surface_objects(*surface_objects);
    segmentation_viz_.Show();
  }

  // Crude assumption: camera pose at demonstration time is approximately the
  // same at imitation time. The proper way to do this would be to store the
  // camera frame transform in the demonstration.
  tg::Graph graph;
  graph.Add("camera", tg::RefFrame("base"),
            cam_interface_.camera_pose().transform);

  // This models the assumption that each demonstration only interacts with an
  // object once.We could / should allow the robot to interact with an object
  // more than once, but we need to continuosly track the objects in that
  // case.
  for (size_t i = 0; i < demo_states.demo_states.size(); ++i) {
    const msgs::DemoState& state = demo_states.demo_states[i];
    for (size_t j = 0; j < state.object_states.size(); ++j) {
      const msgs::ObjectState& os = state.object_states[j];
      if (object_states->find(os.name) == object_states->end()) {
        object_states->insert(
            std::pair<std::string, msgs::ObjectState>(os.name, os));
        graph.Add(os.name, tg::RefFrame("camera"), os.pose);
      }
    }
  }

  // For each object in the demonstration, see if there's a close match in the
  // segmented objects. If so, initialize its position automatically. Otherwise,
  // bring up an interactive marker.
  for (std::map<std::string, msgs::ObjectState>::iterator it =
           object_states->begin();
       it != object_states->end(); ++it) {
    const msgs::ObjectState& obj_state = it->second;
    LazyObjectModel obj_model(obj_state.mesh_name, planning_frame_,
                              obj_state.pose);
    obj_model.set_object_model_cache(&model_cache_);
    geometry_msgs::Vector3 obj_scale = obj_model.scale();

    std::string obj_name(it->first);
    tg::Position obj_position;
    graph.DescribePosition(obj_state.pose.position, tg::Source("camera"),
                           tg::Target("base"), &obj_position);

    geometry_msgs::Pose rough_obj_pose;
    geometry_msgs::Vector3 obj_scale_obs;
    geometry_msgs::Point initial_obj_position = obj_position.point();
    // Flip about y axis to account for mirroring.
    initial_obj_position.y *= -1;
    int obj_index = -1;
    if (surface_objects) {
      obj_index = MatchObject(initial_obj_position, obj_scale, *surface_objects,
                              &rough_obj_pose, &obj_scale_obs);
    }
    dbot_ros_msgs::InitializeObjectGoal init_goal;
    init_goal.frame_id = executor_.planning_frame();
    init_goal.mesh_name = it->second.mesh_name;
    if (obj_index != -1) {
      // surface_perception returns a pose in the center of the object
      // However, our object models have their frames defined in the bottom
      // center of the object. So we shift the pose by half the height.
      tg::Graph graph;
      graph.Add("center frame", tg::RefFrame("planning"), rough_obj_pose);
      tg::Position origin_in_planning;
      graph.DescribePosition(tg::Position(0, 0, -obj_scale.z / 2),
                             tg::Source("center frame"), tg::Target("planning"),
                             &origin_in_planning);
      rough_obj_pose.position = origin_in_planning.point();

      ScoredAlignment obb_pose;
      LazyObjectModel rough_obj_model(obj_state.mesh_name, planning_frame_,
                                      rough_obj_pose);
      rough_obj_model.set_object_model_cache(&model_cache_);
      if (!rough_obj_model.IsCircular()) {
        obb_pose = AlignObject(rough_obj_model,
                               surface_objects->at(0).objects[obj_index]);
      }

      // Get a copy of the model pose, oriented so that its x-axis is aligned
      // with the x-axis of the planning frame.
      ROS_ASSERT(surface_objects->size() == 1);  // MatchObject enforces this
      const surface_perception::Surface& surface =
          surface_objects->at(0).surface;
      Eigen::Quaterniond surface_quat;
      tf::quaternionMsgToEigen(surface.pose_stamped.pose.orientation,
                               surface_quat);
      Eigen::Matrix3d surface_rotation_matrix(surface_quat);
      Eigen::Matrix3d updated_orientation = Eigen::Matrix3d::Identity();
      Eigen::Vector3d surface_x_axis = surface_rotation_matrix.col(0);
      Eigen::Vector3d updated_x(surface_x_axis.x(), 0, surface_x_axis.z());
      updated_x.normalize();
      updated_orientation.col(0) = updated_x;
      updated_orientation.col(2) = surface_rotation_matrix.col(2);
      updated_orientation.col(1) = updated_orientation.col(2).cross(updated_x);

      Eigen::Quaterniond updated_quat(updated_orientation);
      tf::quaternionEigenToMsg(updated_quat, rough_obj_pose.orientation);

      LazyObjectModel axis_aligned_obj_model(obj_state.mesh_name,
                                             planning_frame_, rough_obj_pose);
      axis_aligned_obj_model.set_object_model_cache(&model_cache_);

      ScoredAlignment axis_aligned_pose = AlignObject(
          axis_aligned_obj_model, surface_objects->at(0).objects[obj_index]);

      if (rough_obj_model.IsCircular() ||
          axis_aligned_pose.score < obb_pose.score) {
        ROS_INFO("Using axis-aligned pose. Score: %f vs. %f",
                 axis_aligned_pose.score, obb_pose.score);
        init_goal.initial_pose = axis_aligned_pose.pose;
      } else {
        ROS_INFO("Using oriented pose. Score: %f", obb_pose.score);
        init_goal.initial_pose = obb_pose.pose;
      }
    } else {
      init_goal.initial_pose.orientation.w = 1;
      init_goal.initial_pose.position.x = 1;
      init_goal.initial_pose.position.z = 1;
    }
    ROS_INFO("Initializing pose for object \"%s\"", it->first.c_str());
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
              tg::Transform(tg::Position(-kPregraspDistance, 0, 0),
                            tg::Orientation()));

    tg::Transform pregrasp_in_planning;
    graph.ComputeDescription("pregrasp", tg::RefFrame("planning"),
                             &pregrasp_in_planning);
    MarkerArray pregrasp_markers = gripper_viz_.markers(
        "pregrasp", pregrasp_in_planning.pose(), planning_frame_);
    marker_arr_.markers.insert(marker_arr_.markers.end(),
                               pregrasp_markers.markers.begin(),
                               pregrasp_markers.markers.end());

    tg::Transform grasp_in_planning;
    graph.ComputeDescription("grasp", tg::RefFrame("planning"),
                             &grasp_in_planning);
    MarkerArray grasp_markers = gripper_viz_.markers(
        "grasp", grasp_in_planning.pose(), planning_frame_);
    marker_arr_.markers.insert(marker_arr_.markers.end(),
                               grasp_markers.markers.begin(),
                               grasp_markers.markers.end());
    marker_pub_.publish(marker_arr_);
  } else if (step.type == msgs::Step::UNGRASP) {
  } else if (step.type == msgs::Step::FOLLOW_TRAJECTORY) {
    msgs::ObjectState obj = object_states_[step.object_state.name];
    tg::Graph graph;
    graph.Add("obj", tg::RefFrame("planning"), obj.pose);
    for (size_t i = 0; i < step.ee_trajectory.size(); ++i) {
      const geometry_msgs::Pose& ee_in_obj = step.ee_trajectory[i];
      graph.Add("ee", tg::RefFrame("obj"), ee_in_obj);
      tg::Transform ee_in_planning;
      graph.ComputeDescription("ee", tg::RefFrame("planning"), &ee_in_planning);
      MarkerArray ee_markers =
          gripper_viz_.markers("traj", ee_in_planning.pose(), planning_frame_);
      for (size_t j = 0; j < ee_markers.markers.size(); ++j) {
        ee_markers.markers[j].lifetime = ros::Duration(0.05);
      }
      marker_pub_.publish(ee_markers);
      ros::Duration(0.05).sleep();
    }
  } else if (step.type == msgs::Step::MOVE_TO_POSE) {
    msgs::ObjectState obj = object_states_[step.object_state.name];
    tg::Graph graph;
    graph.Add("obj", tg::RefFrame("planning"), obj.pose);
    graph.Add("dest", tg::RefFrame("obj"), step.ee_trajectory[0]);

    tg::Transform dest_in_planning;
    graph.ComputeDescription("dest", tg::RefFrame("planning"),
                             &dest_in_planning);
    MarkerArray pregrasp_markers =
        gripper_viz_.markers("dest", dest_in_planning.pose(), planning_frame_);
    marker_arr_.markers.insert(marker_arr_.markers.end(),
                               pregrasp_markers.markers.begin(),
                               pregrasp_markers.markers.end());
    marker_pub_.publish(marker_arr_);
  }
}

// Visualizes left and right parts of the slice independently.
void ProgramServer::VisualizeSlice(
    const task_perception_msgs::ProgramSlice& slice) {
  moveit_msgs::RobotTrajectory left_traj;
  left_traj.joint_trajectory = slice.left_traj;
  moveit_msgs::RobotTrajectory right_traj;
  right_traj.joint_trajectory = slice.right_traj;

  moveit::core::RobotStatePtr robot_state = left_group_.getCurrentState();
  trajectory_msgs::JointTrajectory merged_traj;
  trajectory_msgs::JointTrajectoryPoint merged_pt;
  if (left_traj.joint_trajectory.points.size() > 0) {
    trajectory_msgs::JointTrajectoryPoint pt =
        left_traj.joint_trajectory.points[0];
    merged_pt.positions = pt.positions;
    merged_traj.joint_names = left_traj.joint_trajectory.joint_names;
  }
  if (right_traj.joint_trajectory.points.size() > 0) {
    trajectory_msgs::JointTrajectoryPoint pt =
        right_traj.joint_trajectory.points[0];
    merged_pt.positions.insert(merged_pt.positions.end(), pt.positions.begin(),
                               pt.positions.end());
    merged_traj.joint_names.insert(
        merged_traj.joint_names.end(),
        right_traj.joint_trajectory.joint_names.begin(),
        right_traj.joint_trajectory.joint_names.end());
  }
  merged_traj.points.push_back(merged_pt);
  moveit::core::jointTrajPointToRobotState(merged_traj, 0, *robot_state);

  moveit_msgs::DisplayTrajectory display_traj;
  moveit::core::robotStateToRobotStateMsg(*robot_state,
                                          display_traj.trajectory_start);
  display_traj.trajectory.push_back(left_traj);
  display_traj.trajectory.push_back(right_traj);

  traj_pub_.publish(display_traj);
}
}  // namespace pbi
