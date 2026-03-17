// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

// ---------------------------------------------------------------------------
// Coordinate frame conventions
//
// ROS camera optical frame  : x-right, y-down,    z-forward
// ORB-SLAM3 camera frame    : x-right, y-down,    z-forward  (identical)
// ROS body frame (base_link): x-forward, y-left,  z-up
//
// ORB-SLAM3 returns  T_cw  (camera ← world, Sophus::SE3f)
// We expose to ROS   T_wc  (world  ← camera, Eigen::Isometry3d)
//  i.e.  pose_map_camera = T_wc = T_cw.inverse()
//
// For IMU fusion ORB-SLAM3 reports poses in the IMU body frame.
// We compensate with T_body_cam to publish in base_link.
// ---------------------------------------------------------------------------

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Transform.h>

// Forward-declare Sophus without pulling the full header here so that
// compilation units that don't need Sophus don't pay the cost.
namespace Sophus { template<typename T, int N> class SE3; }
using Sophus_SE3f = Sophus::SE3<float, 0>;

#include "isaac_ros_visual_slam_orb/slam_types.hpp"

namespace isaac_ros::visual_slam_orb::conversion
{

// ── Sophus ↔ Eigen ──────────────────────────────────────────────────────────

// T_cw (Sophus, float) → T_wc (Eigen, double)
// The returned isometry maps points from camera to world frame.
Eigen::Isometry3d SophusPoseToEigen(const Sophus_SE3f & T_cw);

// Eigen::Isometry3d → Sophus SE3f
Sophus_SE3f EigenToSophusPose(const Eigen::Isometry3d & T);

// ── Eigen ↔ ROS geometry_msgs ───────────────────────────────────────────────

geometry_msgs::msg::Pose IsometryToRosPose(const Eigen::Isometry3d & T);
Eigen::Isometry3d RosPoseToIsometry(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Transform IsometryToRosTf(const Eigen::Isometry3d & T);
Eigen::Isometry3d RosTfToIsometry(const geometry_msgs::msg::Transform & tf);

// ── tf2 ↔ Eigen ─────────────────────────────────────────────────────────────

Eigen::Isometry3d Tf2ToIsometry(const tf2::Transform & T);
tf2::Transform IsometryToTf2(const Eigen::Isometry3d & T);

// ── CameraInfo → CameraCalib ────────────────────────────────────────────────

// Fills CameraCalib from a ROS CameraInfo message.
// extrinsic T_body_cam must be set separately (from TF tree).
CameraCalib CameraInfoToCalib(
  const sensor_msgs::msg::CameraInfo & info);

// ── IMU ─────────────────────────────────────────────────────────────────────

// Returns linear_acceleration and angular_velocity rotated from
// the IMU optical frame to the body frame.
// If T_body_imu is identity, returns values unchanged.
sensor_msgs::msg::Imu TransformImuToBody(
  const sensor_msgs::msg::Imu & imu,
  const Eigen::Isometry3d & T_body_imu);

// ── Covariance ───────────────────────────────────────────────────────────────

// Rearrange a 6×6 covariance from ORB-SLAM3 order
// [roll,pitch,yaw, x,y,z] → ROS order [x,y,z, roll,pitch,yaw]
// and rotate into the world frame using the given rotation.
Eigen::Matrix<double, 6, 6> ReorderAndRotateCovariance(
  const Eigen::Matrix<float, 6, 6> & cov_orb,
  const Eigen::Matrix3d & R_world_cam);

}  // namespace isaac_ros::visual_slam_orb::conversion
