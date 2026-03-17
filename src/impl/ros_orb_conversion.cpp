// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_visual_slam_orb/impl/ros_orb_conversion.hpp"

#include <sensor_msgs/distortion_models.hpp>

// Pull the full Sophus header here (implementation unit only)
#include <sophus/se3.hpp>

namespace isaac_ros::visual_slam_orb::conversion
{

// ── Sophus ↔ Eigen ──────────────────────────────────────────────────────────

Eigen::Isometry3d SophusPoseToEigen(const Sophus_SE3f & T_cw)
{
  // T_cw: camera ← world.  We want T_wc: world ← camera.
  const Sophus::SE3f T_wc = T_cw.inverse();
  Eigen::Isometry3d result;
  result.linear()      = T_wc.rotationMatrix().cast<double>();
  result.translation() = T_wc.translation().cast<double>();
  return result;
}

Sophus_SE3f EigenToSophusPose(const Eigen::Isometry3d & T)
{
  Eigen::Matrix3f R = T.linear().cast<float>();
  Eigen::Vector3f t = T.translation().cast<float>();
  // Return T_cw (camera ← world) = inverse of the T_wc we received
  return Sophus::SE3f(R, t).inverse();
}

// ── Eigen ↔ ROS geometry_msgs ───────────────────────────────────────────────

geometry_msgs::msg::Pose IsometryToRosPose(const Eigen::Isometry3d & T)
{
  geometry_msgs::msg::Pose p;
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();

  const Eigen::Quaterniond q(T.linear());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

Eigen::Isometry3d RosPoseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  T.linear() = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z).toRotationMatrix();
  return T;
}

geometry_msgs::msg::Transform IsometryToRosTf(const Eigen::Isometry3d & T)
{
  geometry_msgs::msg::Transform tf;
  tf.translation.x = T.translation().x();
  tf.translation.y = T.translation().y();
  tf.translation.z = T.translation().z();

  const Eigen::Quaterniond q(T.linear());
  tf.rotation.x = q.x();
  tf.rotation.y = q.y();
  tf.rotation.z = q.z();
  tf.rotation.w = q.w();
  return tf;
}

Eigen::Isometry3d RosTfToIsometry(const geometry_msgs::msg::Transform & tf)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(
    tf.translation.x, tf.translation.y, tf.translation.z);
  T.linear() = Eigen::Quaterniond(
    tf.rotation.w, tf.rotation.x,
    tf.rotation.y, tf.rotation.z).toRotationMatrix();
  return T;
}

// ── tf2 ↔ Eigen ─────────────────────────────────────────────────────────────

Eigen::Isometry3d Tf2ToIsometry(const tf2::Transform & T)
{
  const tf2::Matrix3x3 rot(T.getRotation());
  Eigen::Matrix3d R;
  for (int i = 0; i < 3; ++i) {
    R(i, 0) = rot.getRow(i).x();
    R(i, 1) = rot.getRow(i).y();
    R(i, 2) = rot.getRow(i).z();
  }
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = R;
  out.translation() = Eigen::Vector3d(T.getOrigin().x(),
                                      T.getOrigin().y(),
                                      T.getOrigin().z());
  return out;
}

tf2::Transform IsometryToTf2(const Eigen::Isometry3d & T)
{
  const Eigen::Quaterniond q(T.linear());
  tf2::Quaternion tq(q.x(), q.y(), q.z(), q.w());
  tf2::Vector3    tt(T.translation().x(),
                     T.translation().y(),
                     T.translation().z());
  return tf2::Transform(tq, tt);
}

// ── CameraInfo → CameraCalib ────────────────────────────────────────────────

CameraCalib CameraInfoToCalib(const sensor_msgs::msg::CameraInfo & info)
{
  CameraCalib c;
  c.width  = info.width;
  c.height = info.height;
  c.fx = info.k[0];
  c.cx = info.k[2];
  c.fy = info.k[4];
  c.cy = info.k[5];
  c.frame_id = info.header.frame_id;

  const auto & dm = info.distortion_model;
  if (dm == sensor_msgs::distortion_models::PLUMB_BOB || dm == "plumb_bob") {
    c.distortion_model = DistortionModel::kBrown5k;
    // k1 k2 p1 p2 k3
    c.dist_coeffs.assign(info.d.begin(),
                         info.d.begin() + std::min<size_t>(5, info.d.size()));
  } else if (dm == sensor_msgs::distortion_models::EQUIDISTANT) {
    c.distortion_model = DistortionModel::kFisheye4;
    // k1 k2 k3 k4
    c.dist_coeffs.assign(info.d.begin(),
                         info.d.begin() + std::min<size_t>(4, info.d.size()));
  } else if (dm == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL) {
    c.distortion_model = DistortionModel::kRationalPolynomial;
    c.dist_coeffs.assign(info.d.begin(), info.d.end());
  } else {
    // Assume pinhole (no distortion)
    c.distortion_model = DistortionModel::kPinhole;
  }

  // T_body_cam is filled externally from the TF tree
  return c;
}

// ── IMU ─────────────────────────────────────────────────────────────────────

sensor_msgs::msg::Imu TransformImuToBody(
  const sensor_msgs::msg::Imu & imu,
  const Eigen::Isometry3d & T_body_imu)
{
  const Eigen::Matrix3d R = T_body_imu.linear();

  const Eigen::Vector3d a_imu(
    imu.linear_acceleration.x,
    imu.linear_acceleration.y,
    imu.linear_acceleration.z);
  const Eigen::Vector3d w_imu(
    imu.angular_velocity.x,
    imu.angular_velocity.y,
    imu.angular_velocity.z);

  const Eigen::Vector3d a_body = R * a_imu;
  const Eigen::Vector3d w_body = R * w_imu;

  sensor_msgs::msg::Imu out = imu;
  out.linear_acceleration.x = a_body.x();
  out.linear_acceleration.y = a_body.y();
  out.linear_acceleration.z = a_body.z();
  out.angular_velocity.x = w_body.x();
  out.angular_velocity.y = w_body.y();
  out.angular_velocity.z = w_body.z();
  return out;
}

// ── Covariance ───────────────────────────────────────────────────────────────

Eigen::Matrix<double, 6, 6> ReorderAndRotateCovariance(
  const Eigen::Matrix<float, 6, 6> & cov_orb,
  const Eigen::Matrix3d & R)
{
  // ORB-SLAM3 covariance order: [roll,pitch,yaw, x,y,z]
  // ROS order:                  [x,y,z, roll,pitch,yaw]
  Eigen::Matrix<double, 6, 6> cov_ros = Eigen::Matrix<double, 6, 6>::Zero();
  cov_ros.block<3, 3>(0, 0) = cov_orb.block<3, 3>(3, 3).cast<double>();  // pos
  cov_ros.block<3, 3>(0, 3) = cov_orb.block<3, 3>(3, 0).cast<double>();
  cov_ros.block<3, 3>(3, 0) = cov_orb.block<3, 3>(0, 3).cast<double>();
  cov_ros.block<3, 3>(3, 3) = cov_orb.block<3, 3>(0, 0).cast<double>();  // rot

  // Rotate into world frame
  Eigen::Matrix<double, 6, 6> block_R = Eigen::Matrix<double, 6, 6>::Zero();
  block_R.block<3, 3>(0, 0) = R;
  block_R.block<3, 3>(3, 3) = R;
  return block_R * cov_ros * block_R.transpose();
}

}  // namespace isaac_ros::visual_slam_orb::conversion
