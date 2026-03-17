// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "isaac_ros_visual_slam_orb/slam_backend.hpp"
#include "isaac_ros_visual_slam_orb/impl/cuda_image_pipeline.hpp"
#include "isaac_ros_visual_slam_orb/impl/cuda_orb_extractor.hpp"

// Forward-declare ORB-SLAM3 types to keep compilation fast in headers
// that include this file but don't need the full ORB-SLAM3 API.
namespace ORB_SLAM3 { class System; }

namespace isaac_ros::visual_slam_orb
{

// ---------------------------------------------------------------------------
// OrbSlam3Backend
//
// Concrete ISlamBackend implementation backed by ORB-SLAM3.
//
// Sensor mode selection:
//   2 cameras,  no IMU  → STEREO
//   2 cameras,  IMU     → IMU_STEREO
//   1 camera,   no IMU  → MONOCULAR   (not recommended for metric scale)
//   1 camera,   IMU     → IMU_MONOCULAR
//
// All heavy work (bundle adjustment, loop closing) runs in ORB-SLAM3's
// own threads.  Track() is synchronous w.r.t. the calling thread.
// ---------------------------------------------------------------------------
class OrbSlam3Backend final : public ISlamBackend
{
public:
  struct Config
  {
    // Paths required by ORB-SLAM3
    std::string vocab_path;     // e.g. /opt/orb_slam3/Vocabulary/ORBvoc.txt
    std::string settings_path;  // e.g. /config/realsense_d456.yaml

    // Feature extractor settings (CUDA or CPU)
    CudaOrbExtractor::Config orb_cfg;

    // Image pre-processing settings (per camera; same config applied to all)
    CudaImagePipeline::Config pipeline_cfg;

    // If true, launch ORB-SLAM3's Pangolin viewer thread.
    // Usually false inside a headless Docker container.
    bool use_viewer{false};

    // Verbosity level forwarded to ORB-SLAM3 (0 = silent, 1 = info, …)
    int verbosity{0};
  };

  explicit OrbSlam3Backend(const Config & cfg);
  ~OrbSlam3Backend() override;

  // ── ISlamBackend ────────────────────────────────────────────────────────

  bool Initialize(
    const std::vector<CameraCalib> & cameras,
    std::optional<ImuCalib> imu = std::nullopt,
    bool rgbd = false) override;

  void Reset() override;
  void Shutdown() override;
  bool IsInitialized() const override;

  TrackingResult Track(
    int64_t timestamp_ns,
    const std::vector<cv::Mat> & images,
    const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> & imus = {}) override;

  bool SaveMap(const std::string & folder_path) override;
  bool LoadMap(const std::string & folder_path) override;
  std::optional<Eigen::Isometry3d> LocalizeInMap(
    const std::string & folder_path,
    const Eigen::Isometry3d & pose_hint) override;

  // support for RGB‑D tracking
  TrackingResult TrackRGBD(
    int64_t timestamp_ns,
    const cv::Mat &color,
    const cv::Mat &depth,
    const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> & imus = {}) override;

  std::vector<KeyframePose> GetAllKeyFramePoses(
    int32_t max_count = -1) const override;

  bool SetSlamPose(const Eigen::Isometry3d & pose_map_camera) override;

private:
  // Build the ORB-SLAM3 YAML settings string dynamically from CameraCalib
  // so the caller doesn't need to maintain a separate YAML file for
  // intrinsics (only the algorithm parameters need to be in settings_path).
  bool WriteSettingsYaml(
    const std::vector<CameraCalib> & cameras,
    const std::optional<ImuCalib> & imu);

  // Convert ORB-SLAM3 tracking state to our enum
  static TrackingState OrbStateToTrackingState(int orb_state);

  // Collect map points and observations from ORB-SLAM3 after tracking
  void FillVisualizationData(TrackingResult & result) const;

  Config cfg_;
  std::unique_ptr<ORB_SLAM3::System> system_;

  // Per-camera pipeline (CUDA preprocessing)
  std::vector<std::unique_ptr<CudaImagePipeline>> pipelines_;

  // Shared CUDA ORB extractor (ORB-SLAM3 uses its own internal extractor,
  // but we override it via the Settings mechanism or a custom extractor slot)
  std::unique_ptr<CudaOrbExtractor> extractor_;

  // Calibration cached at Initialize() time
  std::vector<CameraCalib> cameras_;
  std::optional<ImuCalib>  imu_calib_;

  // Path of the temp settings YAML generated at Initialize()
  std::string generated_settings_path_;

  mutable std::mutex mutex_;
  std::atomic<bool>  initialized_{false};
  // whether backend was initialised in RGB‑D mode
  bool rgbd_mode_{false};
};

}  // namespace isaac_ros::visual_slam_orb
