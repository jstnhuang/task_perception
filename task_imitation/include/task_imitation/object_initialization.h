#ifndef _PBI_OBJECT_INITIALIZATION_H_
#define _PBI_OBJECT_INITIALIZATION_H_

#include "boost/optional.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_robot/camera_interface.h"
#include "surface_perception/surface_objects.h"
#include "task_perception/lazy_object_model.h"

namespace pbi {
// Gets a point cloud from the camera interface in the base frame.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloudInBase(
    const rapid::PointCloudCameraInterface& cam_interface);

// Crop a point cloud using ROS parameters.
// Assumes parameters are named like: param_prefix + "min_x" for min_x, max_x,
// min_y, max_y, min_z, and max_z params.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CropCloudUsingParams(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
    const std::string& param_prefix);

boost::optional<std::vector<surface_perception::SurfaceObjects> >
DetectTabletopObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// Find which object in surface_objects best matches the object whose scale is
// obj_scale. Matches must be "good enough," meaning each dimension matches by
// some amount. If more than one "good enough" matches are found, the best match
// will be returned (as defined by the norm of the difference in scale). Returns
// the index of the matched object in surface_objects if a "good enough" match
// was found, -1 otherwise. The pose and scale of the best match will be output.
int MatchObject(
    const geometry_msgs::Point& initial_obj_position,
    const geometry_msgs::Vector3& obj_scale,
    const std::vector<surface_perception::SurfaceObjects>& surface_objects,
    geometry_msgs::Pose* pose, geometry_msgs::Vector3* scale);

// Do a fine-grained alignment using ICP.
// object_model: The model of the object to align
// initial_pose: The initial pose of the object to align
// target_object: The object to align to
geometry_msgs::Pose AlignObject(
    const LazyObjectModel& object_model,
    const surface_perception::Object& target_object);

}  // namespace pbi

#endif  // _PBI_OBJECT_INITIALIZATION_H_
