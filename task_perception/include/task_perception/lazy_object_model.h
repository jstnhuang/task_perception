#ifndef _PBI_LAZY_OBJECT_MODEL_H_
#define _PBI_LAZY_OBJECT_MODEL_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"

namespace pbi {
// Uses lazily computation to compute object models, normals, and KD-trees.
class LazyObjectModel {
 public:
  typedef std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>
      ObjectModelCache;

  // Constructor
  //
  // mesh_name is the name of the mesh file, like "pringles_1k.obj". This class
  // looks for the mesh in the object_models directory of the ROS package
  // object_meshes. However, note that we actually require .pcd files. That is,
  // the mesh_name is "pringles_1k.obj", but the actual mesh that's loaded is
  // "pringles_1k.pcd". This is because dbot uses .obj files, but when we
  // convert the .obj to a point cloud, we save the point cloud as .pcd.
  LazyObjectModel(const std::string& mesh_name, const std::string& frame_id,
                  const geometry_msgs::Pose& pose);
  void set_object_model_cache(ObjectModelCache* cache);

  // Returns the object model (in the frame defined by the object model).
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetObjectModel() const;

  // Returns the object model, transformed according to the pose.
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetObjectCloud() const;
  pcl::PointCloud<pcl::PointNormal>::Ptr GetObjectCloudWithNormals() const;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr GetObjectTree() const;

  geometry_msgs::Pose pose() const;
  geometry_msgs::Vector3 scale() const;

 private:
  std::string mesh_name_;
  std::string frame_id_;
  geometry_msgs::Pose pose_;

  mutable ObjectModelCache* cache_;  // No ownership
  const std::string kPackagePath_;

  mutable pcl::PointCloud<pcl::PointXYZ>::Ptr object_model_;
  mutable pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_;
  mutable pcl::PointCloud<pcl::PointNormal>::Ptr object_cloud_with_normals_;
  mutable pcl::search::KdTree<pcl::PointXYZ>::Ptr object_tree_;
  mutable geometry_msgs::Vector3 scale_;  // x < 0 if uninitialized
};

pcl::PointCloud<pcl::PointXYZ>::Ptr LoadModel(const std::string& mesh_path);

// Replace path/to/file.obj to path/to/file.pcd.
std::string ReplaceObjWithPcd(const std::string& path);
}  // namespace pbi

#endif  // _PBI_LAZY_OBJECT_MODEL_H_
