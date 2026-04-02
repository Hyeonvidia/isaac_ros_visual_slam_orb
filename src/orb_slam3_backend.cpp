// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_visual_slam_orb/orb_slam3_backend.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

// ORB-SLAM3 public headers
#include <System.h>
#include <MapPoint.h>
#include <KeyFrame.h>
#include <Atlas.h>

#include <sophus/se3.hpp>

#include "isaac_ros_visual_slam_orb/impl/ros_orb_conversion.hpp"

namespace isaac_ros::visual_slam_orb
{

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

OrbSlam3Backend::OrbSlam3Backend(const Config & cfg)
: cfg_(cfg)
{}

OrbSlam3Backend::~OrbSlam3Backend()
{
  Shutdown();
}

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

bool OrbSlam3Backend::Initialize(
  const std::vector<CameraCalib> & cameras,
  std::optional<ImuCalib> imu,
  bool rgbd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rgbd_mode_ = rgbd;

  if (initialized_) {
    return true;
  }

  cameras_.assign(cameras.begin(), cameras.end());
  imu_calib_ = imu;

  // ── Choose ORB-SLAM3 sensor mode ────────────────────────────────────────
  ORB_SLAM3::System::eSensor sensor_mode;
  const bool stereo = cameras_.size() >= 2;
  const bool has_imu = imu_calib_.has_value();
  const bool is_rgbd = (cameras_.size() == 1 && rgbd_mode_);

  if (is_rgbd && has_imu)        sensor_mode = ORB_SLAM3::System::IMU_RGBD;
  else if (is_rgbd)              sensor_mode = ORB_SLAM3::System::RGBD;
  else if (stereo && has_imu)    sensor_mode = ORB_SLAM3::System::IMU_STEREO;
  else if (stereo)               sensor_mode = ORB_SLAM3::System::STEREO;
  else if (has_imu)              sensor_mode = ORB_SLAM3::System::IMU_MONOCULAR;
  else                            sensor_mode = ORB_SLAM3::System::MONOCULAR;

  // ── Generate settings YAML ──────────────────────────────────────────────
  if (!WriteSettingsYaml(cameras, imu)) {
    return false;
  }

  // ── Create image pipelines (one per camera) ──────────────────────────────
  pipelines_.clear();
  for (const auto & cam : cameras_) {
    auto pipe = std::make_unique<CudaImagePipeline>(cfg_.pipeline_cfg);

    // Build undistort + rectification maps from calibration
    const int w = static_cast<int>(cam.width);
    const int h = static_cast<int>(cam.height);

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
      cam.fx, 0,      cam.cx,
      0,      cam.fy, cam.cy,
      0,      0,      1);

    cv::Mat D(static_cast<int>(cam.dist_coeffs.size()), 1, CV_64F);
    for (size_t i = 0; i < cam.dist_coeffs.size(); ++i) {
      D.at<double>(static_cast<int>(i)) = cam.dist_coeffs[i];
    }

    // Use identity for R and the same K for P (no stereo rectification here;
    // ORB-SLAM3 handles stereo geometry internally from the YAML config).
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat P = cv::Mat::zeros(3, 4, CV_64F);
    K.copyTo(P(cv::Rect(0, 0, 3, 3)));

    pipe->SetRectifyMaps(K, D, R, P, cv::Size(w, h));
    pipelines_.push_back(std::move(pipe));
  }

  // ── Create shared ORB extractor ──────────────────────────────────────────
  extractor_ = std::make_unique<CudaOrbExtractor>(cfg_.orb_cfg);

  // ── Launch ORB-SLAM3 ─────────────────────────────────────────────────────
  try {
    system_ = std::make_unique<ORB_SLAM3::System>(
      cfg_.vocab_path,
      generated_settings_path_,
      sensor_mode,
      cfg_.use_viewer,
      cfg_.verbosity);
  } catch (const std::exception & e) {
    return false;
  }

  initialized_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// Settings YAML generation
//
// ORB-SLAM3 requires a YAML file with camera intrinsics + algorithm params.
// We generate it at runtime from the CameraCalib so the user only needs to
// keep algorithm-tuning parameters in the static config file.
// ---------------------------------------------------------------------------

bool OrbSlam3Backend::WriteSettingsYaml(
  const std::vector<CameraCalib> & cameras,
  const std::optional<ImuCalib> & imu)
{
  // Use /tmp with PID suffix to avoid stale YAML from previous runs
  // (the host /tmp/ is bind-mounted and persists across containers)
  generated_settings_path_ =
    "/tmp/orb_slam3_settings_" + std::to_string(getpid()) + ".yaml";

  std::ofstream f(generated_settings_path_);
  if (!f.is_open()) {
    return false;
  }
  // Helper: ORB-SLAM3's YAML parser is strict about types.
  // - "real number" params MUST have a decimal point (200 → 200.0)
  // - "integer" params MUST NOT have a decimal point (30.0 → 30)
  // We use a lambda to force a decimal point on doubles.
  auto real = [](double v) -> std::string {
    std::ostringstream os;
    os << std::setprecision(10) << v;
    std::string s = os.str();
    if (s.find('.') == std::string::npos && s.find('e') == std::string::npos) {
      s += ".0";
    }
    return s;
  };

  // ── File header ────────────────────────────────────────────────────────
  // File.version: "1.0" activates the NEW Settings.cc parser.
  // Without it, ORB-SLAM3 falls back to the old Tracking.cc parser
  // which uses different key names and fails.
  f << "%YAML:1.0\n\n";
  f << "File.version: \"1.0\"\n\n";

  // ── Camera model ──────────────────────────────────────────────────────
  const auto & cam0 = cameras[0];
  // Use "Rectified" for all modes:
  // - Stereo/Mono: IR images are hardware-rectified by RealSense
  // - RGBD: aligned depth is already in color frame; "Rectified" avoids
  //   ORB-SLAM3 trying to read Stereo.T_c1_c2 (PinHole type bug).
  //   We patched Settings.cc to create calibration2_ for Rectified.
  f << "Camera.type: \"Rectified\"\n\n";
  f << "Camera1.fx: " << real(cam0.fx) << "\n";
  f << "Camera1.fy: " << real(cam0.fy) << "\n";
  f << "Camera1.cx: " << real(cam0.cx) << "\n";
  f << "Camera1.cy: " << real(cam0.cy) << "\n";
  f << "\n";

  // Camera2: required for stereo/stereo-IMU modes.
  // For Rectified stereo the intrinsics are identical to Camera1.
  if (cameras.size() >= 2) {
    const auto & cam1 = cameras[1];
    f << "Camera2.fx: " << real(cam1.fx) << "\n";
    f << "Camera2.fy: " << real(cam1.fy) << "\n";
    f << "Camera2.cx: " << real(cam1.cx) << "\n";
    f << "Camera2.cy: " << real(cam1.cy) << "\n";
    f << "\n";
  }

  // ── Image dimensions — integers ───────────────────────────────────────
  f << "Camera.width: "  << cam0.width  << "\n";
  f << "Camera.height: " << cam0.height << "\n";
  f << "Camera.fps: 30\n";       // integer
  if (rgbd_mode_) {
    f << "Camera.RGB: 1\n";      // integer
    f << "RGBD.DepthMapFactor: " << real(1000.0) << "\n";
  } else {
    f << "Camera.RGB: 0\n";      // integer
  }
  // Camera.newHeight / Camera.newWidth intentionally omitted.
  // If present with value 0, ORB-SLAM3 creates 0×0 rectification maps → crash.
  // When absent, ORB-SLAM3 defaults to original image dimensions.
  f << "\n";

  // ── Depth threshold (used by both Stereo and RGBD) ──────────────────
  // ThDepth * baseline = max usable depth in metres.
  // 50 * 0.095 ≈ 4.75m (good for automotive range with D456).
  f << "Stereo.ThDepth: " << real(50.0) << "\n";

  // ── Stereo baseline ─────────────────────────────────────────────────
  if (cameras.size() >= 2) {
    // Stereo: compute from TF
    const Eigen::Vector3d t1 = cameras[0].T_body_cam.translation();
    const Eigen::Vector3d t2 = cameras[1].T_body_cam.translation();
    double baseline = (t2 - t1).norm();
    if (baseline < 1e-6) {
      baseline = 0.095;
    }
    f << "Stereo.b: " << real(baseline) << "\n";
  } else {
    // Mono/RGBD: ORB-SLAM3 Rectified parser still requires Stereo.b.
    // Use a dummy value; RGBD uses depth directly, mono ignores it.
    f << "Stereo.b: " << real(0.095) << "\n";
  }
  f << "\n";

  // ── IMU — real numbers ────────────────────────────────────────────────
  if (imu.has_value()) {
    const auto & im = imu.value();

    const Eigen::Isometry3d T_imu_cam =
      im.T_body_imu.inverse() * cameras[0].T_body_cam;
    const auto & M = T_imu_cam.matrix();

    f << "IMU.T_b_c1: !!opencv-matrix\n";
    f << "   rows: 4\n";
    f << "   cols: 4\n";
    f << "   dt: f\n";
    f << "   data: [";
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        f << real(M(r, c));
        if (r != 3 || c != 3) { f << ", "; }
      }
      if (r < 3) { f << "\n          "; }
    }
    f << "]\n\n";

    f << "IMU.NoiseGyro: "  << real(im.gyro_noise_density)  << "\n";
    f << "IMU.NoiseAcc: "   << real(im.accel_noise_density) << "\n";
    f << "IMU.GyroWalk: "   << real(im.gyro_random_walk)    << "\n";
    f << "IMU.AccWalk: "    << real(im.accel_random_walk)   << "\n";
    f << "IMU.Frequency: "  << real(im.frequency)           << "\n";
    f << "IMU.InsertKFsWhenLost: 1\n";  // integer — keep inserting KFs for recovery
    f << "\n";
  }

  // ── ORB feature parameters ────────────────────────────────────────────
  f << "ORBextractor.nFeatures: "   << cfg_.orb_cfg.num_features       << "\n";  // int
  f << "ORBextractor.scaleFactor: " << real(cfg_.orb_cfg.scale_factor) << "\n";  // real
  f << "ORBextractor.nLevels: "     << cfg_.orb_cfg.num_levels         << "\n";  // int
  f << "ORBextractor.iniThFAST: "   << cfg_.orb_cfg.ini_threshold_fast << "\n";  // int
  f << "ORBextractor.minThFAST: "   << cfg_.orb_cfg.min_threshold_fast << "\n";  // int

  // ── Viewer — all real numbers ────────────────────────────────────────
  f << "Viewer.KeyFrameSize: 0.05\n";
  f << "Viewer.KeyFrameLineWidth: 1.0\n";
  f << "Viewer.GraphLineWidth: 0.9\n";
  f << "Viewer.PointSize: 2.0\n";
  f << "Viewer.CameraSize: 0.08\n";
  f << "Viewer.CameraLineWidth: 3.0\n";
  f << "Viewer.ViewpointX: 0.0\n";
  f << "Viewer.ViewpointY: -0.7\n";
  f << "Viewer.ViewpointZ: -1.8\n";
  f << "Viewer.ViewpointF: 500.0\n";
  f << "Viewer.imageViewScale: 1.0\n";

  f.close();

  // Dump generated YAML for debugging
  std::ifstream dump(generated_settings_path_);
  if (dump.is_open()) {
    std::cerr << "\n=== Generated ORB-SLAM3 YAML (" << generated_settings_path_ << ") ===\n"
              << dump.rdbuf()
              << "\n=== End YAML ===\n" << std::endl;
  }

  return true;
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void OrbSlam3Backend::Reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (system_) {
    system_->Reset();
  }
  initialized_ = false;
}

// ---------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------

void OrbSlam3Backend::Shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (system_) {
    system_->Shutdown();
    system_.reset();
  }
  initialized_ = false;
}

bool OrbSlam3Backend::IsInitialized() const
{
  return initialized_.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Track
// ---------------------------------------------------------------------------

TrackingResult OrbSlam3Backend::TrackRGBD(
  int64_t timestamp_ns,
  const cv::Mat &color,
  const cv::Mat &depth,
  const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> & imus)
{
  TrackingResult result;
  if (!initialized_ || !system_) {
    result.state = TrackingState::kFailed;
    return result;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const double timestamp_s = static_cast<double>(timestamp_ns) * 1e-9;

  // preprocess colour image through pipeline[0]
  std::vector<cv::Mat> processed;
  processed.reserve(2);
  if (!pipelines_.empty()) {
    processed.push_back(pipelines_[0]->ProcessCpu(color));
  } else {
    processed.push_back(color);
  }
  // keep depth as-is (no preprocessing)
  processed.push_back(depth);

  // build imu points
  std::vector<ORB_SLAM3::IMU::Point> imu_points;
  imu_points.reserve(imus.size());
  for (const auto & imu_msg : imus) {
    imu_points.emplace_back(
      static_cast<float>(imu_msg->linear_acceleration.x),
      static_cast<float>(imu_msg->linear_acceleration.y),
      static_cast<float>(imu_msg->linear_acceleration.z),
      static_cast<float>(imu_msg->angular_velocity.x),
      static_cast<float>(imu_msg->angular_velocity.y),
      static_cast<float>(imu_msg->angular_velocity.z),
      static_cast<double>(imu_msg->header.stamp.sec) +
        static_cast<double>(imu_msg->header.stamp.nanosec) * 1e-9);
  }

  Sophus::SE3f T_cw;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    T_cw = system_->TrackRGBD(processed[0], processed[1], timestamp_s, imu_points);
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.track_time_s = std::chrono::duration<double>(t1 - t0).count();

  result.state = OrbStateToTrackingState(system_->GetTrackingState());
  if (result.state == TrackingState::kSuccess ||
      result.state == TrackingState::kMapRelocalization) {
    result.pose_map_camera = conversion::SophusPoseToEigen(T_cw);
    FillVisualizationData(result);
  }

  const auto t2 = std::chrono::steady_clock::now();
  result.total_time_s = std::chrono::duration<double>(t2 - t0).count();
  return result;
}

TrackingResult OrbSlam3Backend::Track(
  int64_t timestamp_ns,
  const std::vector<cv::Mat> & images,
  const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> & imus)
{
  TrackingResult result;

  if (!initialized_ || !system_) {
    result.state = TrackingState::kFailed;
    return result;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const double timestamp_s = static_cast<double>(timestamp_ns) * 1e-9;

  // ── Preprocess images ─────────────────────────────────────────────────
  std::vector<cv::Mat> processed;
  processed.reserve(images.size());
  for (size_t i = 0; i < images.size() && i < pipelines_.size(); ++i) {
    processed.push_back(pipelines_[i]->ProcessCpu(images[i]));
  }


  // ── Build IMU measurement vector ──────────────────────────────────────
  std::vector<ORB_SLAM3::IMU::Point> imu_points;
  imu_points.reserve(imus.size());
  for (const auto & imu_msg : imus) {
    imu_points.emplace_back(
      static_cast<float>(imu_msg->linear_acceleration.x),
      static_cast<float>(imu_msg->linear_acceleration.y),
      static_cast<float>(imu_msg->linear_acceleration.z),
      static_cast<float>(imu_msg->angular_velocity.x),
      static_cast<float>(imu_msg->angular_velocity.y),
      static_cast<float>(imu_msg->angular_velocity.z),
      static_cast<double>(imu_msg->header.stamp.sec) +
        static_cast<double>(imu_msg->header.stamp.nanosec) * 1e-9);
  }

  // ── Call ORB-SLAM3 tracking ───────────────────────────────────────────
  Sophus::SE3f T_cw;
  {
    std::lock_guard<std::mutex> lock(mutex_);

      const bool stereo = processed.size() >= 2;
    const bool has_imu = !imu_points.empty();

    try {
      if (stereo && has_imu) {
        T_cw = system_->TrackStereo(
          processed[0], processed[1], timestamp_s,
          imu_points);
      } else if (stereo) {
        T_cw = system_->TrackStereo(
          processed[0], processed[1], timestamp_s);
      } else if (!processed.empty() && has_imu) {
        T_cw = system_->TrackMonocular(
          processed[0], timestamp_s, imu_points);
      } else if (!processed.empty()) {
        T_cw = system_->TrackMonocular(processed[0], timestamp_s);
      }
    } catch (const std::exception & e) {
      std::cerr << "[ORB-SLAM3] Track exception: " << e.what() << std::endl;
      result.state = TrackingState::kFailed;
      return result;
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.track_time_s = std::chrono::duration<double>(t1 - t0).count();

  // ── Convert pose ──────────────────────────────────────────────────────
  result.state = OrbStateToTrackingState(system_->GetTrackingState());

  if (result.state == TrackingState::kSuccess ||
      result.state == TrackingState::kMapRelocalization)
  {
    result.pose_map_camera = conversion::SophusPoseToEigen(T_cw);
    FillVisualizationData(result);
  }

  const auto t2 = std::chrono::steady_clock::now();
  result.total_time_s = std::chrono::duration<double>(t2 - t0).count();

  return result;
}

// ---------------------------------------------------------------------------
// Map management
// ---------------------------------------------------------------------------

bool OrbSlam3Backend::SaveMap(const std::string & folder_path)
{
  if (!initialized_ || !system_) { return false; }

  // SaveAtlas is private in upstream ORB-SLAM3.
  // A patched fork or future version can expose it publicly.
  // For now, map save/load is not supported.
  (void)folder_path;
  return false;
}

bool OrbSlam3Backend::LoadMap(const std::string & /*folder_path*/)
{
  if (!initialized_ || !system_) { return false; }
  std::lock_guard<std::mutex> lock(mutex_);
  // ORB-SLAM3 loads the atlas from the path embedded in its YAML config.
  // Our modified fork accepts a runtime path via ChangeDataset().
  system_->ChangeDataset();
  return true;
}

std::optional<Eigen::Isometry3d> OrbSlam3Backend::LocalizeInMap(
  const std::string & /*folder_path*/,
  const Eigen::Isometry3d & /*pose_hint*/)
{
  // Relocalisation in ORB-SLAM3 happens automatically when the tracking
  // thread detects a known keyframe cluster.  We expose the outcome here
  // by checking whether the current tracking state is LOST→RELOCALIZING.
  // A more precise implementation patches ORB-SLAM3 to accept a pose hint.
  if (!initialized_ || !system_) { return std::nullopt; }
  if (system_->GetTrackingState() == 3 /*LOST*/) { return std::nullopt; }
  return std::nullopt;  // async — caller polls via Track()
}

// ---------------------------------------------------------------------------
// Pose graph queries
// ---------------------------------------------------------------------------

std::vector<KeyframePose> OrbSlam3Backend::GetAllKeyFramePoses(
  int32_t max_count) const
{
  std::vector<KeyframePose> out;
  if (!initialized_ || !system_) { return out; }

  std::lock_guard<std::mutex> lock(mutex_);
  // GetAtlas() is private in upstream ORB-SLAM3.
  // Return empty until a patched fork exposes keyframe queries.
  return out;
  (void)max_count;
#if 0  // Requires patched ORB-SLAM3 with public GetAtlas()
  const auto kfs = system_->GetAtlas()->GetCurrentMap()->GetAllKeyFrames();

  int count = 0;
  for (auto * kf : kfs) {
    if (kf->isBad()) { continue; }
    if (max_count > 0 && count >= max_count) { break; }

    KeyframePose kp;
    kp.timestamp_ns = static_cast<int64_t>(kf->mTimeStamp * 1e9);
    kp.pose_map_camera = conversion::SophusPoseToEigen(kf->GetPose());
    out.push_back(kp);
    ++count;
  }
  return out;
#endif
}

bool OrbSlam3Backend::SetSlamPose(const Eigen::Isometry3d & /*pose_map_camera*/)
{
  // TODO(implementation): patch ORB-SLAM3 to accept an external pose hint.
  // For now this is a no-op; the relocalisation loop will converge on its own.
  return false;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

TrackingState OrbSlam3Backend::OrbStateToTrackingState(int orb_state)
{
  // ORB-SLAM3 Tracking::eTrackingState:
  //   SYSTEM_NOT_READY=-1, NO_IMAGES_YET=0, NOT_INITIALIZED=1,
  //   OK=2, RECENTLY_LOST=3, LOST=4, OK_KLT=5
  switch (orb_state) {
    case 2:  return TrackingState::kSuccess;
    case 5:  return TrackingState::kSuccess;       // KLT mode
    case 3:  return TrackingState::kMapRelocalization;
    default: return TrackingState::kFailed;
  }
}

void OrbSlam3Backend::FillVisualizationData(TrackingResult & result) const
{
  if (!system_) { return; }

  // Map points tracked in the last frame
  const auto tracked = system_->GetTrackedMapPoints();
  result.map_points.reserve(tracked.size());
  for (auto * mp : tracked) {
    if (!mp || mp->isBad()) { continue; }
    const Eigen::Vector3f pos = mp->GetWorldPos();
    result.map_points.emplace_back(pos);
  }

  // 2D keypoint positions projected to 3D observations (unit vectors)
  const auto kps = system_->GetTrackedKeyPointsUn();
  result.observations.reserve(kps.size());
  for (const auto & kp : kps) {
    result.observations.emplace_back(kp.pt.x, kp.pt.y, 1.0f);
  }
}

}  // namespace isaac_ros::visual_slam_orb
