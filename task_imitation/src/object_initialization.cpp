#include "task_imitation/object_initialization.h"

#include <limits.h>
#include <math.h>
#include <algorithm>
#include <utility>

#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "pcl/filters/crop_box.h"
#include "pcl/registration/icp.h"
#include "pcl_ros/transforms.h"
#include "rapid_ros/params.h"
#include "rapid_utils/pcl_typedefs.h"
#include "rapid_utils/vector3.hpp"
#include "ros/ros.h"
#include "surface_perception/segmentation.h"

namespace pbi {
using boost::optional;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloudInBase(
    const rapid::PointCloudCameraInterface& cam_interface) {
  rapid::PointCloudC::Ptr cloud = cam_interface.cloud();
  geometry_msgs::TransformStamped cam_in_base = cam_interface.camera_pose();
  Eigen::Affine3d transform;
  tf::transformMsgToEigen(cam_in_base.transform, transform);
  rapid::PointCloudC::Ptr cloud_in_base(new rapid::PointCloudC);
  pcl::transformPointCloud(*cloud, *cloud_in_base, transform);
  cloud_in_base->header.frame_id = cam_in_base.header.frame_id;
  return cloud_in_base;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CropCloudUsingParams(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
    const std::string& param_prefix) {
  pcl::CropBox<rapid::PointC> crop;
  crop.setInputCloud(cloud);
  double min_x = rapid::GetDoubleParamOrThrow(param_prefix + "min_x");
  double min_y = rapid::GetDoubleParamOrThrow(param_prefix + "min_y");
  double min_z = rapid::GetDoubleParamOrThrow(param_prefix + "min_z");
  double max_x = rapid::GetDoubleParamOrThrow(param_prefix + "max_x");
  double max_y = rapid::GetDoubleParamOrThrow(param_prefix + "max_y");
  double max_z = rapid::GetDoubleParamOrThrow(param_prefix + "max_z");
  Eigen::Vector4f min_pt;
  min_pt << min_x, min_y, min_z, 1;
  crop.setMin(min_pt);
  Eigen::Vector4f max_pt;
  max_pt << max_x, max_y, max_z, 1;
  crop.setMax(max_pt);
  rapid::PointCloudC::Ptr cropped_cloud(new rapid::PointCloudC);
  crop.filter(*cropped_cloud);
  return cropped_cloud;
}

optional<std::vector<surface_perception::SurfaceObjects> >
DetectTabletopObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  surface_perception::Segmentation seg;
  seg.set_input_cloud(cloud);
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
  seg.set_max_point_distance(
      rapid::GetDoubleParamOrThrow("surface_segmentation/max_point_distance"));
  std::vector<surface_perception::SurfaceObjects> surface_objects;
  bool segment_success = seg.Segment(&surface_objects);
  if (segment_success) {
    return surface_objects;
  } else {
    return boost::none;
  }
}

int MatchObject(
    const geometry_msgs::Point& initial_obj_position,
    const geometry_msgs::Vector3& obj_scale,
    const std::vector<surface_perception::SurfaceObjects>& surface_objects,
    geometry_msgs::Pose* pose, geometry_msgs::Vector3* scale) {
  if (surface_objects.size() != 1) {
    ROS_ERROR("Expected to find exactly one surface, found %zu",
              surface_objects.size());
    return -1;
  }

  // Pick the one closest in size
  // If there are multiple objects close in size (ambiguity threshold), pick the
  // one closest to the demonstrated starting position.
  // Final check that the object dimensions are not completely terrible.

  double size_ambiguity_threshold =
      rapid::GetDoubleParamOrThrow("task_imitation/size_ambiguity_threshold");
  double match_dim_tolerance =
      rapid::GetDoubleParamOrThrow("task_imitation/match_dim_tolerance");
  const surface_perception::SurfaceObjects& surface = surface_objects[0];
  std::vector<std::pair<double, int> > size_match_scores;
  for (size_t j = 0; j < surface.objects.size(); ++j) {
    const surface_perception::Object& object = surface.objects[j];
    double dim_dx = fabs(object.dimensions.x - obj_scale.x);
    double dim_dy = fabs(object.dimensions.y - obj_scale.y);
    double dim_dz = fabs(object.dimensions.z - obj_scale.z);
    // Check that the sizes are approximately equal in at least two dimensions
    int num_matching_dims = (dim_dx < match_dim_tolerance) +
                            (dim_dy < match_dim_tolerance) +
                            (dim_dz < match_dim_tolerance);
    if (num_matching_dims < 2) {
      ROS_INFO(
          "Model with dims %f %f %f did not match object with dims %f %f %f",
          obj_scale.x, obj_scale.y, obj_scale.z, object.dimensions.x,
          object.dimensions.y, object.dimensions.z);
      continue;
    }
    double sq_dim_dist = dim_dx * dim_dx + dim_dy * dim_dy + dim_dz * dim_dz;
    size_match_scores.push_back(std::make_pair<double, int>(sq_dim_dist, j));
  }

  std::sort(size_match_scores.begin(), size_match_scores.end());
  double best_sq_dim_dist = size_match_scores[0].first;

  if (size_match_scores.size() == 0) {
    return -1;
  }

  const surface_perception::Object& best_object =
      surface.objects[size_match_scores[0].second];
  int best_index = size_match_scores[0].second;
  *pose = best_object.pose_stamped.pose;
  *scale = best_object.dimensions;

  double best_sq_dist = std::numeric_limits<double>::max();
  for (size_t j = 0; j < size_match_scores.size(); ++j) {
    const std::pair<double, int>& size_match_score = size_match_scores[j];
    const surface_perception::Object& object =
        surface.objects[size_match_scores[j].second];

    ROS_INFO(
        "Best object has ambiguity score with rank #%zu (dims %f %f %f) of %f",
        j + 1, object.dimensions.x, object.dimensions.y, object.dimensions.z,
        best_sq_dim_dist / size_match_score.first);
    if (best_sq_dim_dist / size_match_score.first > size_ambiguity_threshold) {
      double dx =
          fabs(initial_obj_position.x - object.pose_stamped.pose.position.x);
      double dy =
          fabs(initial_obj_position.y - object.pose_stamped.pose.position.y);
      double dz =
          fabs(initial_obj_position.z - object.pose_stamped.pose.position.z);
      double sq_dist = dx * dx + dy * dy + dz * dz;
      if (sq_dist < best_sq_dist) {
        best_sq_dist = sq_dist;
        best_index = size_match_scores[j].second;
        *pose = object.pose_stamped.pose;
        *scale = object.dimensions;
      }
    } else {
      break;
    }
  }

  return best_index;
}

geometry_msgs::Pose AlignObject(
    const LazyObjectModel& object_model,
    const surface_perception::Object& target_object) {
  rapid::PointCloudP::Ptr obj_cloud = object_model.GetObjectCloud();

  // Extract target cloud and convert to PointXYZ
  rapid::PointCloudP::Ptr obs_cloud(new rapid::PointCloudP);
  obs_cloud->header = target_object.cloud->header;
  for (size_t i = 0; i < target_object.indices->indices.size(); ++i) {
    int index = target_object.indices->indices[i];
    const pcl::PointXYZRGB& color_pt = target_object.cloud->at(index);
    pcl::PointXYZ pt;
    pt.x = color_pt.x;
    pt.y = color_pt.y;
    pt.z = color_pt.z;
    obs_cloud->push_back(pt);
  }

  // We actually align the observation to the model rather than vice versa
  // because ICP works better when you align smaller objects to larger ones. We
  // see less of the observation than we do of the full model.
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(obs_cloud);
  icp.setInputTarget(obj_cloud);
  icp.setMaximumIterations(100);
  rapid::PointCloudP aligned;
  icp.align(aligned);

  if (icp.hasConverged()) {
    ROS_INFO("Aligned object with score %f", icp.getFitnessScore());
    Eigen::Matrix4f aligned_in_obs = icp.getFinalTransformation();
    Eigen::Matrix4d aligned_in_obs_d = aligned_in_obs.cast<double>();
    Eigen::Affine3d aligned_affine(aligned_in_obs_d);
    Eigen::Affine3d model_pose;
    tf::poseMsgToEigen(object_model.pose(), model_pose);

    Eigen::Affine3d updated_model_pose = aligned_affine.inverse() * model_pose;
    geometry_msgs::Pose result;
    tf::poseEigenToMsg(updated_model_pose, result);
    return result;
  } else {
    ROS_WARN("Failed to align object. Fitness score: %f",
             icp.getFitnessScore());
    return object_model.pose();
  }
}
}  // namespace pbi
