// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "isaac_ros_visual_slam_orb/slam_types.hpp"

namespace isaac_ros::visual_slam_orb
{

// ---------------------------------------------------------------------------
// ISlamBackend — pure virtual interface
//
// Replaces all CUVSLAM_* API calls.  The VisualSlamNode only uses this
// interface, making it trivial to swap cuVSLAM ↔ ORB-SLAM3 (or any future
// backend) by constructing a different concrete implementation.
// ---------------------------------------------------------------------------
class ISlamBackend
{
public:
  virtual ~ISlamBackend() = default;

  // ── Lifecycle ──────────────────────────────────────────────────────────

  // Called once after camera calibrations and (optionally) IMU calibration
  // are known.  Must be called before Track*().
  // Returns false and logs error if initialization fails.
  // If |rgbd| is true the caller will feed TrackRGBD() and the cameras
  // list should contain exactly one camera.  Backends may use this hint to
  // write depth-related entries in the YAML settings.
  virtual bool Initialize(
    const std::vector<CameraCalib> & cameras,
    std::optional<ImuCalib> imu = std::nullopt,
    bool rgbd = false) = 0;

  // Reset to initial state (same as the ROS Reset service).
  // Must re-initialize before tracking again after Reset().
  virtual void Reset() = 0;

  // Orderly shutdown (flush threads, release GPU memory, …).
  virtual void Shutdown() = 0;

  // True after Initialize() succeeds, false after Reset() / before Init.
  virtual bool IsInitialized() const = 0;

  // ── Core tracking ──────────────────────────────────────────────────────

  // Process a synchronised set of images (CPU side).
  // |images| must have the same count as the cameras passed to Initialize().
  // |imus|   contains all IMU samples between the previous and current frame.
  // Returns the estimated camera pose and auxiliary visualisation data.
  virtual TrackingResult Track(
    int64_t timestamp_ns,
    const std::vector<cv::Mat> & images,
    const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> & imus = {}) = 0;

  // Same as Track() but for an RGB‑D camera: supplied color image and
  // depth map (same resolution).  Depth should be CV_32F or CV_16U where
  // values represent meters (RGBD mode inside ORB‑SLAM3 expects meters).
  virtual TrackingResult TrackRGBD(
    int64_t timestamp_ns,
    const cv::Mat &color,
    const cv::Mat &depth,
    const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> & imus = {})
  {
    (void)timestamp_ns; (void)color; (void)depth; (void)imus;
    throw std::runtime_error("RGBD tracking not implemented");
  }

  // ── Map management ─────────────────────────────────────────────────────

  // Serialise the current map to |folder_path|.
  // May block until serialisation is complete.
  virtual bool SaveMap(const std::string & folder_path) = 0;

  // Deserialise a previously saved map from |folder_path| and replace the
  // current map.  After loading the node is still in odometry-only mode
  // until LocalizeInMap() succeeds.
  virtual bool LoadMap(const std::string & folder_path) = 0;

  // Attempt to localise the current camera in a previously saved map.
  // |pose_hint| is an initial guess for map_T_camera.
  // Returns the refined map_T_camera on success, nullopt on failure.
  virtual std::optional<Eigen::Isometry3d> LocalizeInMap(
    const std::string & folder_path,
    const Eigen::Isometry3d & pose_hint) = 0;

  // ── Pose graph queries ─────────────────────────────────────────────────

  // Return up to |max_count| keyframe poses (most recent first).
  // |max_count| <= 0 means "all".
  virtual std::vector<KeyframePose> GetAllKeyFramePoses(
    int32_t max_count = -1) const = 0;

  // Override the current SLAM pose with an externally known pose.
  // Useful when receiving an initial pose from /initialpose topic.
  virtual bool SetSlamPose(const Eigen::Isometry3d & pose_map_camera) = 0;
};

}  // namespace isaac_ros::visual_slam_orb
