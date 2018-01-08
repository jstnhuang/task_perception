#include "task_perception/camera_info_camera_provider.h"

#include "Eigen/Dense"
#include "dbot/camera_data_provider.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

namespace pbi {
CameraInfoCameraProvider::CameraInfoCameraProvider(
    const sensor_msgs::CameraInfo& camera_info, int downsampling_factor)
    : camera_info_(camera_info),
      downsampling_factor_(downsampling_factor),
      native_resolution_() {
  native_resolution_.height = camera_info.height;
  native_resolution_.width = camera_info.width;
}

Eigen::MatrixXd CameraInfoCameraProvider::depth_image() const {
  ROS_ERROR("Call to unused method CameraInfoCameraProvider::depth_image()");
  return Eigen::MatrixXd::Zero(1, 1);
}

Eigen::VectorXd CameraInfoCameraProvider::depth_image_vector() const {
  ROS_ERROR("Call to unused method CameraInfoCameraProvider::depth_image()");
  return Eigen::Vector3d::Zero(1);
}

Eigen::Matrix3d CameraInfoCameraProvider::camera_matrix() const {
  Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Zero();
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      camera_matrix(row, col) = camera_info_.K[col + row * 3];
    }
  }
  camera_matrix.topLeftCorner(2, 3) /= downsampling_factor_;
  return camera_matrix;
}

std::string CameraInfoCameraProvider::frame_id() const {
  return camera_info_.header.frame_id;
}

int CameraInfoCameraProvider::downsampling_factor() const {
  return downsampling_factor_;
}

dbot::CameraData::Resolution CameraInfoCameraProvider::native_resolution()
    const {
  return native_resolution_;
}

}  // namespace pbi
