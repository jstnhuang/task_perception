#include "task_perception/lazy_object_model.h"

#include "Eigen/Eigen"
#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "transform_graph/graph.h"

#include "task_perception/object_model_cache.h"
#include "task_perception/pcl_typedefs.h"
#include "task_perception/shape_detection.h"

namespace tg = transform_graph;

namespace pbi {
LazyObjectModel::LazyObjectModel(const std::string& mesh_name,
                                 const std::string& frame_id,
                                 const geometry_msgs::Pose& mesh_pose)
    : mesh_name_(mesh_name),
      frame_id_(frame_id),
      mesh_pose_(mesh_pose),
      center_pose_(),
      cache_(NULL),
      kPackagePath_(ros::package::getPath("object_meshes") + "/object_models/"),
      object_model_(),
      object_cloud_(),
      object_cloud_with_normals_(),
      object_tree_(),
      scale_() {
  scale_.x = -1;
}

void LazyObjectModel::set_object_model_cache(ObjectModelCache* cache) {
  cache_ = cache;
}

PointCloudP::Ptr LazyObjectModel::GetObjectModel() const {
  ROS_ASSERT(cache_ != NULL);
  if (!object_model_) {
    // Get from cache if possible
    if (cache_ != NULL &&
        cache_->clouds.find(mesh_name_) != cache_->clouds.end()) {
      object_model_ = cache_->clouds.at(mesh_name_);
      return object_model_;
    }
    // Otherwise, load the model and insert into the cache if possible
    std::string path = kPackagePath_ + mesh_name_;
    path = ReplaceObjWithPcd(path);
    object_model_ = LoadModel(path);
    if (cache_ != NULL) {
      cache_->clouds.insert(
          std::pair<std::string, PointCloudP::Ptr>(mesh_name_, object_model_));
    }
    return object_model_;
  } else {
    return object_model_;
  }
}

PointCloudP::Ptr LazyObjectModel::GetObjectCloud() const {
  if (!object_cloud_) {
    PointCloudP::Ptr object_model = GetObjectModel();
    Eigen::Affine3d object_transform;
    tf::poseMsgToEigen(mesh_pose_, object_transform);
    object_cloud_.reset(new PointCloudP);
    pcl::transformPointCloud(*object_model, *object_cloud_, object_transform);
    object_cloud_->header.frame_id = frame_id_;
  }
  return object_cloud_;
}

PointCloudN::Ptr LazyObjectModel::GetObjectCloudWithNormals() const {
  if (!object_cloud_with_normals_) {
    PointCloudP::Ptr object_cloud = GetObjectCloud();
    KdTreeP::Ptr tree = GetObjectTree();
    pcl::NormalEstimationOMP<PointP, pcl::Normal> ne;
    ne.setInputCloud(object_cloud);
    ne.setSearchMethod(tree);
    // Assumes that PCD models have a voxel size of 0.01
    ne.setRadiusSearch(0.012);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    object_cloud_with_normals_.reset(new PointCloudN);
    pcl::concatenateFields(*object_cloud, *normals,
                           *object_cloud_with_normals_);
    object_cloud_with_normals_->header.frame_id = frame_id_;
  }
  return object_cloud_with_normals_;
}

KdTreeP::Ptr LazyObjectModel::GetObjectTree() const {
  if (!object_tree_) {
    object_tree_.reset(new pcl::search::KdTree<PointP>);
    object_tree_->setInputCloud(GetObjectCloud());
  }
  return object_tree_;
}

geometry_msgs::Pose LazyObjectModel::mesh_pose() const { return mesh_pose_; }

geometry_msgs::Pose LazyObjectModel::center_pose() const {
  if (!center_pose_) {
    tg::Graph graph;
    graph.Add("pose", tg::RefFrame("world"), mesh_pose_);
    geometry_msgs::Vector3 obj_scale = scale();
    tg::Transform obj_center_pose;
    graph.DescribePose(
        tg::Transform(tg::Position(0, 0, obj_scale.z / 2), tg::Orientation()),
        tg::Source("pose"), tg::Target("world"), &obj_center_pose);
    center_pose_ = obj_center_pose.pose();
  }
  return *center_pose_;
}

geometry_msgs::Vector3 LazyObjectModel::scale() const {
  if (scale_.x < 0) {
    PointCloudP::Ptr model = GetObjectModel();
    PointP min, max;
    pcl::getMinMax3D(*model, min, max);
    scale_.x = max.x - min.x;
    scale_.y = max.y - min.y;
    scale_.z = max.z - min.z;
  }
  return scale_;
}

bool LazyObjectModel::IsCircular() const {
  ROS_ASSERT(cache_ != NULL);
  if (cache_ != NULL &&
      cache_->is_circular.find(mesh_name_) != cache_->is_circular.end()) {
    return cache_->is_circular.at(mesh_name_);
  }
  bool is_circular = ::pbi::IsCircular(GetObjectModel());
  if (cache_ != NULL) {
    cache_->is_circular[mesh_name_] = is_circular;
    if (is_circular) {
      ROS_INFO("%s is circular", mesh_name_.c_str());
    } else {
      ROS_INFO("%s is not circular", mesh_name_.c_str());
    }
  }
  return is_circular;
}

PointCloudP::Ptr LoadModel(const std::string& mesh_path) {
  PointCloudP::Ptr object_model(new PointCloudP);
  pcl::io::loadPCDFile(mesh_path, *object_model);
  ROS_INFO("Loaded mesh %s with %ld points", mesh_path.c_str(),
           object_model->size());
  return object_model;
}

std::string ReplaceObjWithPcd(const std::string& path) {
  std::string base_path = path.substr(0, path.size() - 4);
  return base_path + ".pcd";
}
}  // namespace pbi
