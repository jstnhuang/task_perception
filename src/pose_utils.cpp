#include "task_perception/pose_utils.h"

#include <math.h>

#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"

namespace pbi {
double LinearDistance(const geometry_msgs::Pose& start,
                      const geometry_msgs::Pose& end) {
  double dx = end.position.x - start.position.x;
  double dy = end.position.y - start.position.y;
  double dz = end.position.z - start.position.z;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

double AngularDistance(const geometry_msgs::Pose& start,
                       const geometry_msgs::Pose& end) {
  Eigen::Affine3d start_mat;
  tf::poseMsgToEigen(start, start_mat);
  Eigen::Affine3d end_mat;
  tf::poseMsgToEigen(end, end_mat);

  Eigen::Vector3d start_x = start_mat.rotation().col(0);
  Eigen::Vector3d end_x = end_mat.rotation().col(0);
  Eigen::Quaterniond rotation =
      Eigen::Quaterniond::FromTwoVectors(start_x, end_x);
  Eigen::AngleAxisd aa(rotation);
  return aa.angle();
}
}  // namespace pbi
