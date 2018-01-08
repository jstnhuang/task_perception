#ifndef _PBI_CAMERA_INFO_CAMERA_PROVIDER_H_
#define _PBI_CAMERA_INFO_CAMERA_PROVIDER_H_

#include "Eigen/Dense"
#include "dbot/camera_data_provider.h"
#include "sensor_msgs/CameraInfo.h"

namespace pbi {
class CameraInfoCameraProvider : public dbot::CameraDataProvider {
 public:
  CameraInfoCameraProvider(const sensor_msgs::CameraInfo& camera_info,
                           int downsampling_factor);

  Eigen::MatrixXd depth_image() const;
  Eigen::VectorXd depth_image_vector() const;
  Eigen::Matrix3d camera_matrix() const;
  std::string frame_id() const;
  int downsampling_factor() const;
  dbot::CameraData::Resolution native_resolution() const;

 private:
  sensor_msgs::CameraInfo camera_info_;
  int downsampling_factor_;
  dbot::CameraData::Resolution native_resolution_;
};
}  // namespace pbi

#endif  // _PBI_CAMERA_INFO_CAMERA_PROVIDER_H_
