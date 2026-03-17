// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace isaac_ros::visual_slam_orb
{

// ---------------------------------------------------------------------------
// Camera intrinsics + extrinsics (replaces CUVSLAM_Camera)
// ---------------------------------------------------------------------------
enum class DistortionModel : uint8_t
{
  kPinhole = 0,       // 0 distortion params
  kBrown5k = 1,       // plumb_bob: k1,k2,p1,p2,k3
  kFisheye4 = 2,      // equidistant: k1,k2,k3,k4
  kRationalPolynomial, // k1,k2,p1,p2,k3,k4,k5,k6
};

struct CameraCalib
{
  uint32_t width{0};
  uint32_t height{0};
  double fx{0.0}, fy{0.0}, cx{0.0}, cy{0.0};
  std::vector<double> dist_coeffs;          // ordered per distortion_model
  DistortionModel distortion_model{DistortionModel::kBrown5k};

  // Rigid transform: body_frame <- camera_optical_frame
  // i.e. T_body_cam  (SE3: rotation + translation)
  Eigen::Isometry3d T_body_cam{Eigen::Isometry3d::Identity()};

  // ROS frame id (for TF lookup / CameraInfo header)
  std::string frame_id;
};

// ---------------------------------------------------------------------------
// IMU noise parameters (replaces CUVSLAM_ImuCalibration)
// ---------------------------------------------------------------------------
struct ImuCalib
{
  double gyro_noise_density{0.000244};       // rad / (s * sqrt(Hz))
  double gyro_random_walk{0.000019393};      // rad / (s^2 * sqrt(Hz))
  double accel_noise_density{0.001862};      // m / (s^2 * sqrt(Hz))
  double accel_random_walk{0.003};           // m / (s^3 * sqrt(Hz))
  double frequency{200.0};                   // Hz

  // Rigid transform: body_frame <- imu_frame
  Eigen::Isometry3d T_body_imu{Eigen::Isometry3d::Identity()};

  std::string frame_id;
};

// ---------------------------------------------------------------------------
// Tracking state (mirrors cuVSLAM VisualSlamStatus vo_state)
// ---------------------------------------------------------------------------
enum class TrackingState : uint8_t
{
  kUnknown = 0,
  kSuccess = 1,
  kFailed  = 2,
  kMapRelocalization = 3,   // successfully relocated in loaded map
};

// ---------------------------------------------------------------------------
// Per-frame tracking result (replaces all CUVSLAM_Track outputs)
// ---------------------------------------------------------------------------
struct TrackingResult
{
  TrackingState state{TrackingState::kUnknown};

  // Pose in map frame: map_T_camera (SE3)
  Eigen::Isometry3d pose_map_camera{Eigen::Isometry3d::Identity()};

  // 6x6 covariance in ROS order: [x,y,z, rx,ry,rz]
  Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};

  // 3D map points in world frame (for landmark visualisation)
  std::vector<Eigen::Vector3f> map_points;

  // 2D->3D observations in camera frame (for observation visualisation)
  std::vector<Eigen::Vector3f> observations;

  // Timing
  double track_time_s{0.0};
  double total_time_s{0.0};
};

// ---------------------------------------------------------------------------
// Keyframe pose entry (for GetAllPoses service)
// ---------------------------------------------------------------------------
struct KeyframePose
{
  int64_t timestamp_ns{0};
  Eigen::Isometry3d pose_map_camera{Eigen::Isometry3d::Identity()};
};

}  // namespace isaac_ros::visual_slam_orb
