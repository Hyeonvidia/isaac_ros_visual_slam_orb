// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_visual_slam_orb/visual_slam_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "isaac_ros_visual_slam_orb/orb_slam3_backend.hpp"
#include "isaac_ros_visual_slam_orb/impl/ros_orb_conversion.hpp"

namespace isaac_ros::visual_slam_orb
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
VisualSlamNode::VisualSlamNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("visual_slam_node", options),
  // ── Parameter declarations (same names as isaac_ros_visual_slam) ────────
  num_cameras_(declare_parameter<int>("num_cameras", 2)),
  sync_matching_threshold_ms_(declare_parameter<double>("sync_matching_threshold_ms", 5.0)),
  min_num_images_(declare_parameter<int>("min_num_images", 1)),
  enable_imu_fusion_(declare_parameter<bool>("enable_imu_fusion", false)),
  rectified_images_(declare_parameter<bool>("rectified_images", true)),
  enable_image_denoising_(declare_parameter<bool>("enable_image_denoising", true)),
  enable_localization_n_mapping_(declare_parameter<bool>("enable_localization_n_mapping", true)),
  enable_ground_constraint_in_odometry_(
    declare_parameter<bool>("enable_ground_constraint_in_odometry", false)),
  publish_map_to_odom_tf_(declare_parameter<bool>("publish_map_to_odom_tf", true)),
  publish_odom_to_base_tf_(declare_parameter<bool>("publish_odom_to_base_tf", true)),
  invert_map_to_odom_tf_(declare_parameter<bool>("invert_map_to_odom_tf", false)),
  invert_odom_to_base_tf_(declare_parameter<bool>("invert_odom_to_base_tf", false)),
  enable_slam_visualization_(declare_parameter<bool>("enable_slam_visualization", true)),
  enable_observations_view_(declare_parameter<bool>("enable_observations_view", true)),
  enable_landmarks_view_(declare_parameter<bool>("enable_landmarks_view", true)),
  override_publishing_stamp_(declare_parameter<bool>("override_publishing_stamp", false)),
  path_max_size_(declare_parameter<int>("path_max_size", 1024)),
  image_buffer_size_(declare_parameter<int>("image_buffer_size", 10)),
  imu_buffer_size_(declare_parameter<int>("imu_buffer_size", 100)),
  image_jitter_threshold_ms_(declare_parameter<double>("image_jitter_threshold_ms", 34.0)),
  imu_jitter_threshold_ms_(declare_parameter<double>("imu_jitter_threshold_ms", 10.0)),
  map_frame_(declare_parameter<std::string>("map_frame", "map")),
  odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
  base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
  camera_optical_frames_(
    declare_parameter<std::vector<std::string>>("camera_optical_frames", {})),
  imu_frame_(declare_parameter<std::string>("imu_frame", "")),
  sensor_mode_str_(declare_parameter<std::string>("sensor_mode", "stereo")),
  depth_topic_(declare_parameter<std::string>("depth_topic", "visual_slam/depth")),
  // ORB-SLAM3 specific
  orb_vocab_path_(declare_parameter<std::string>(
    "orb_vocab_path", "/opt/orb_slam3/Vocabulary/ORBvoc.txt")),
  orb_settings_path_(declare_parameter<std::string>("orb_settings_path", "")),
  orb_num_features_(declare_parameter<int>("orb_num_features", 1000)),
  orb_scale_factor_(static_cast<float>(declare_parameter<double>("orb_scale_factor", 1.2))),
  orb_num_levels_(declare_parameter<int>("orb_num_levels", 8)),
  gyro_noise_density_(declare_parameter<double>("gyro_noise_density", 0.000244)),
  gyro_random_walk_(declare_parameter<double>("gyro_random_walk", 0.000019393)),
  accel_noise_density_(declare_parameter<double>("accel_noise_density", 0.001862)),
  accel_random_walk_(declare_parameter<double>("accel_random_walk", 0.003)),
  calibration_frequency_(declare_parameter<double>("calibration_frequency", 200.0)),
  sync_()  // initialised below after sensor_mode is known
{
  // ── Interpret sensor_mode parameter ──────────────────────────────────
  if (sensor_mode_str_ == "mono")           sensor_mode_ = SensorMode::kMono;
  else if (sensor_mode_str_ == "mono-imu")  sensor_mode_ = SensorMode::kMonoImu;
  else if (sensor_mode_str_ == "stereo")    sensor_mode_ = SensorMode::kStereo;
  else if (sensor_mode_str_ == "stereo-imu")sensor_mode_ = SensorMode::kStereoImu;
  else if (sensor_mode_str_ == "rgbd")      sensor_mode_ = SensorMode::kRgbd;
  else if (sensor_mode_str_ == "rgbd-imu")  sensor_mode_ = SensorMode::kRgbdImu;
  else {
    RCLCPP_WARN(get_logger(), "Unknown sensor_mode '%s', defaulting to stereo",
                sensor_mode_str_.c_str());
    sensor_mode_ = SensorMode::kStereo;
  }

  switch (sensor_mode_) {
    case SensorMode::kMono:
    case SensorMode::kMonoImu:
    case SensorMode::kRgbd:
    case SensorMode::kRgbdImu:
      effective_num_cameras_ = 1;
      break;
    case SensorMode::kStereo:
    case SensorMode::kStereoImu:
      effective_num_cameras_ = 2;
      break;
  }
  rgbd_mode_ = (sensor_mode_ == SensorMode::kRgbd ||
                sensor_mode_ == SensorMode::kRgbdImu);

  if ((sensor_mode_ == SensorMode::kStereo ||
       sensor_mode_ == SensorMode::kStereoImu) && num_cameras_ < 2) {
    RCLCPP_WARN(get_logger(),
      "sensor_mode indicates stereo but num_cameras=%u, overriding for sync",
      num_cameras_);
  }

  // ── Initialise synchroniser with correct stream count ────────────────
  {
    uint32_t sync_topics = effective_num_cameras_ + (rgbd_mode_ ? 1u : 0u);
    // min_num_images must match the total number of expected streams:
    // - stereo: 2 images (left + right)
    // - rgbd: 2 streams (color + depth)
    // - mono: 1 image
    const int min_images = static_cast<int>(sync_topics);
    sync_ = std::make_unique<Synchronizer>(sync_topics,
                         static_cast<int>(1e6 * sync_matching_threshold_ms_),
                         min_images,
                         image_buffer_size_);
    sync_->RegisterCallback(
      std::bind(&VisualSlamNode::OnSyncedImages, this,
                std::placeholders::_1, std::placeholders::_2));
  }

  // depth subscription (rgbd modes only)
  if (rgbd_mode_) {
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisualSlamNode::OnDepth, this, std::placeholders::_1));
  }

  // ── TF ────────────────────────────────────────────────────────────────
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);


  // ── Publishers ────────────────────────────────────────────────────────
  status_pub_       = create_publisher<
    isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>(
    "visual_slam/status", 10);
  vo_pose_pub_      = create_publisher<geometry_msgs::msg::PoseStamped>(
    "visual_slam/tracking/vo_pose", 10);
  vo_pose_cov_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "visual_slam/tracking/vo_pose_covariance", 10);
  odometry_pub_     = create_publisher<nav_msgs::msg::Odometry>(
    "visual_slam/tracking/odometry", 10);
  vo_path_pub_      = create_publisher<nav_msgs::msg::Path>(
    "visual_slam/tracking/vo_path", 10);
  slam_path_pub_    = create_publisher<nav_msgs::msg::Path>(
    "visual_slam/tracking/slam_path", 10);
  diagnostics_pub_  = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 10);
  observations_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "visual_slam/vis/observations_cloud", 5);
  landmarks_pub_    = create_publisher<sensor_msgs::msg::PointCloud2>(
    "visual_slam/vis/landmarks_cloud", 5);
  pose_graph_pub_   = create_publisher<geometry_msgs::msg::PoseArray>(
    "visual_slam/vis/pose_graph_nodes", 5);

  // ── Subscribers ───────────────────────────────────────────────────────
  for (uint32_t i = 0; i < effective_num_cameras_; ++i) {
    const std::string img_topic  = "visual_slam/image_" + std::to_string(i);
    const std::string info_topic = "visual_slam/camera_info_" + std::to_string(i);

    image_subs_.push_back(
      create_subscription<sensor_msgs::msg::Image>(
        img_topic, rclcpp::SensorDataQoS(),
        [this, i](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
          OnImage(static_cast<int>(i), msg);
        }));

    camera_info_subs_.push_back(
      create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        [this, i](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
          OnCameraInfo(static_cast<int>(i), msg);
        }));
  }

  if (enable_imu_fusion_) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "visual_slam/imu", rclcpp::SensorDataQoS(),
      std::bind(&VisualSlamNode::OnImu, this, std::placeholders::_1));
  }

  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&VisualSlamNode::OnInitialPose, this, std::placeholders::_1));

  // ── Services ──────────────────────────────────────────────────────────
  reset_srv_ = create_service<isaac_ros_visual_slam_interfaces::srv::Reset>(
    "visual_slam/reset",
    std::bind(&VisualSlamNode::HandleReset, this,
              std::placeholders::_1, std::placeholders::_2));

  get_poses_srv_ = create_service<isaac_ros_visual_slam_interfaces::srv::GetAllPoses>(
    "visual_slam/get_all_poses",
    std::bind(&VisualSlamNode::HandleGetAllPoses, this,
              std::placeholders::_1, std::placeholders::_2));

  set_pose_srv_ = create_service<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>(
    "visual_slam/set_slam_pose",
    std::bind(&VisualSlamNode::HandleSetSlamPose, this,
              std::placeholders::_1, std::placeholders::_2));

  save_map_srv_ = create_service<isaac_ros_visual_slam_interfaces::srv::FilePath>(
    "visual_slam/save_map",
    std::bind(&VisualSlamNode::HandleSaveMap, this,
              std::placeholders::_1, std::placeholders::_2));

  load_map_srv_ = create_service<isaac_ros_visual_slam_interfaces::srv::FilePath>(
    "visual_slam/load_map",
    std::bind(&VisualSlamNode::HandleLoadMap, this,
              std::placeholders::_1, std::placeholders::_2));

  localize_srv_ = create_service<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap>(
    "visual_slam/localize_in_map",
    std::bind(&VisualSlamNode::HandleLocalizeInMap, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(),
    "VisualSlamNode (ORB-SLAM3 backend) initialised. "
    "Mode=%s, expecting %u camera(s)%s%s.",
    sensor_mode_str_.c_str(), effective_num_cameras_,
    enable_imu_fusion_ ? " + IMU" : "",
    rgbd_mode_ ? " + depth" : "");
}

VisualSlamNode::~VisualSlamNode()
{
  if (slam_backend_) {
    slam_backend_->Shutdown();
  }
}

// ---------------------------------------------------------------------------
// Subscribers
// ---------------------------------------------------------------------------

void VisualSlamNode::OnImage(
  int index, const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!slam_backend_ || !slam_backend_->IsInitialized()) {
    TryInitialize();
    // Don't return — still feed images to sync so they're ready
    // when initialization completes.
    if (!slam_backend_ || !slam_backend_->IsInitialized()) {
      return;  // Still not ready, discard
    }
  }
  const int64_t ts_ns =
    static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
    msg->header.stamp.nanosec;
  sync_->AddMessage(index, ts_ns, *msg);
}

void VisualSlamNode::OnCameraInfo(
  int index, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (initial_camera_info_.find(index) == initial_camera_info_.end()) {
    initial_camera_info_[index] = msg;
    RCLCPP_INFO(get_logger(), "Received camera info for camera %d.", index);
    TryInitialize();
  }
}

void VisualSlamNode::OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  if (!initial_imu_msg_) {
    initial_imu_msg_ = msg;
    TryInitialize();
  }
  std::lock_guard<std::mutex> lock(imu_mutex_);
  pending_imu_.push_back(msg);
  // Keep buffer bounded
  while (pending_imu_.size() > static_cast<size_t>(imu_buffer_size_)) {
    pending_imu_.erase(pending_imu_.begin());
  }
}

void VisualSlamNode::OnDepth(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!slam_backend_ || !slam_backend_->IsInitialized()) {
    // depth not required for initialization
  }
  const int64_t ts_ns =
    static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
    msg->header.stamp.nanosec;
  // depth stream index is effective_num_cameras_
  sync_->AddMessage(static_cast<int>(effective_num_cameras_), ts_ns, *msg);
}

void VisualSlamNode::OnInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  if (!slam_backend_ || !slam_backend_->IsInitialized()) { return; }
  const Eigen::Isometry3d pose =
    conversion::RosPoseToIsometry(msg->pose.pose);
  slam_backend_->SetSlamPose(pose);
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

bool VisualSlamNode::IsReadyForInitialization() const
{
  if (enable_imu_fusion_ && !initial_imu_msg_) { return false; }
  // mono/stereo cameras must have info; depth is not required for init
  return static_cast<uint32_t>(initial_camera_info_.size()) >= effective_num_cameras_;
}

void VisualSlamNode::TryInitialize()
{
  if (slam_backend_ && slam_backend_->IsInitialized()) { return; }
  if (!IsReadyForInitialization()) { return; }

  RCLCPP_INFO(get_logger(), "Initialising ORB-SLAM3 backend…");

  // ── Build camera calibrations ────────────────────────────────────────
  std::vector<CameraCalib> cameras;
  cameras.reserve(effective_num_cameras_);

  for (uint32_t i = 0; i < effective_num_cameras_; ++i) {
    const auto & info_msg = initial_camera_info_.at(i).value();
    CameraCalib c = conversion::CameraInfoToCalib(*info_msg);

    // Resolve camera optical frame name
    const std::string frame =
      (i < camera_optical_frames_.size())
        ? camera_optical_frames_[i]
        : info_msg->header.frame_id;
    c.frame_id = frame;

    // Look up T_body_cam from TF tree
    try {
      c.T_body_cam = LookupTransform(base_frame_, frame);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(),
        "TF lookup base→%s failed (%s). Using identity.", frame.c_str(), e.what());
    }

    cameras.push_back(c);
  }

  // ── Build IMU calibration (optional) ────────────────────────────────
  std::optional<ImuCalib> imu_calib;
  if (enable_imu_fusion_) {
    ImuCalib imu;
    imu.gyro_noise_density  = gyro_noise_density_;
    imu.gyro_random_walk    = gyro_random_walk_;
    imu.accel_noise_density = accel_noise_density_;
    imu.accel_random_walk   = accel_random_walk_;
    imu.frequency           = calibration_frequency_;
    imu.frame_id            = imu_frame_;

    if (!imu_frame_.empty()) {
      try {
        imu.T_body_imu = LookupTransform(base_frame_, imu_frame_);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(),
          "TF lookup base→imu failed (%s). Using identity.", e.what());
      }
    }
    imu_calib = imu;
  }

  // ── Store base→camera transform for coordinate frame conversion ──────
  // ORB-SLAM3 world is in camera optical convention (x-right, y-down, z-forward).
  // ROS base_link is (x-forward, y-left, z-up).
  // T_base_cam_ captures this rotation from the TF tree.
  if (!cameras.empty()) {
    T_base_cam_ = cameras[0].T_body_cam;
    T_cam_base_ = T_base_cam_.inverse();
  }

  // ── Construct ORB-SLAM3 backend ──────────────────────────────────────
  OrbSlam3Backend::Config cfg;
  cfg.vocab_path    = orb_vocab_path_;
  cfg.settings_path = orb_settings_path_;
  cfg.use_viewer    = declare_parameter<bool>("enable_viewer", false);
  cfg.verbosity     = 0;
  cfg.orb_cfg.num_features      = orb_num_features_;
  cfg.orb_cfg.scale_factor      = orb_scale_factor_;
  cfg.orb_cfg.num_levels        = orb_num_levels_;
  cfg.pipeline_cfg.enable_denoising = enable_image_denoising_;
  cfg.pipeline_cfg.rectify          = !rectified_images_;  // skip if already rectified

  slam_backend_ = std::make_unique<OrbSlam3Backend>(cfg);

  if (!slam_backend_->Initialize(cameras, imu_calib, rgbd_mode_)) {
    RCLCPP_ERROR(get_logger(), "ORB-SLAM3 backend initialisation failed.");
    slam_backend_.reset();
    return;
  }

  RCLCPP_INFO(get_logger(), "ORB-SLAM3 backend ready.");
}

// ---------------------------------------------------------------------------
// Synchronised image callback → Track
// ---------------------------------------------------------------------------

void VisualSlamNode::OnSyncedImages(
  int64_t timestamp_ns,
  const std::vector<std::pair<int, sensor_msgs::msg::Image>> & idx_images)
{
  if (!slam_backend_ || !slam_backend_->IsInitialized()) { return; }

  // ── Convert ROS images to cv::Mat ────────────────────────────────────
  std::vector<cv::Mat> cv_images;
  cv_images.reserve(effective_num_cameras_);
  cv::Mat depth_mat;

  for (const auto & [idx, img_msg] : idx_images) {
    if (rgbd_mode_ && idx == static_cast<int>(effective_num_cameras_)) {
      // last stream is depth
      depth_mat = cv_bridge::toCvShare(
        std::make_shared<sensor_msgs::msg::Image>(img_msg),
        img_msg.encoding)->image.clone();
      // convert to 32‑bit float meters if needed
      if (depth_mat.type() != CV_32F) {
        cv::Mat tmp;
        depth_mat.convertTo(tmp, CV_32F);
        depth_mat = tmp;
      }
    } else {
      auto cv_ptr = cv_bridge::toCvShare(
        std::make_shared<sensor_msgs::msg::Image>(img_msg), "mono8");
      cv_images.push_back(cv_ptr->image.clone());
    }
  }

  // ── Collect IMU messages up to current image timestamp ────────────────
  // ORB-SLAM3 needs IMU measurements between consecutive frames.
  // Only take messages with timestamp <= current image, leave the rest.
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imus;
  if (enable_imu_fusion_) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    auto it = pending_imu_.begin();
    while (it != pending_imu_.end()) {
      const int64_t imu_ts =
        static_cast<int64_t>((*it)->header.stamp.sec) * 1'000'000'000LL +
        (*it)->header.stamp.nanosec;
      if (imu_ts <= timestamp_ns) {
        imus.push_back(*it);
        it = pending_imu_.erase(it);
      } else {
        break;  // IMU messages are chronological
      }
    }
  }

  // ── Track ─────────────────────────────────────────────────────────────
  TrackingResult result;
  if (rgbd_mode_) {
    result = slam_backend_->TrackRGBD(timestamp_ns, cv_images[0], depth_mat, imus);
  } else {
    result = slam_backend_->Track(timestamp_ns, cv_images, imus);
  }

  // ── Publish ───────────────────────────────────────────────────────────
  const rclcpp::Time stamp = EffectiveStamp(
    rclcpp::Time(
      static_cast<int32_t>(timestamp_ns / 1'000'000'000LL),
      static_cast<uint32_t>(timestamp_ns % 1'000'000'000LL),
      RCL_ROS_TIME));

  PublishAll(stamp, result);
}

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

rclcpp::Time VisualSlamNode::EffectiveStamp(const rclcpp::Time & stamp) const
{
  return override_publishing_stamp_ ? now() : stamp;
}

void VisualSlamNode::PublishAll(
  const rclcpp::Time & stamp,
  const TrackingResult & result)
{
  PublishStatus(stamp, result);

  if (result.state == TrackingState::kSuccess ||
      result.state == TrackingState::kMapRelocalization)
  {
    PublishTf(stamp, result.pose_map_camera);
    PublishOdometry(stamp, result);

    if (enable_slam_visualization_) {
      PublishVisualization(stamp, result);
    }
  }
}

void VisualSlamNode::PublishTf(
  const rclcpp::Time & stamp,
  const Eigen::Isometry3d & pose_map_camera)
{
  // ── Coordinate frame conversion ────────────────────────────────────────
  // ORB-SLAM3 returns T_wc (world ← camera) in optical frame convention
  // (x-right, y-down, z-forward). ROS base_link uses (x-forward, y-left, z-up).
  //
  // The similarity transform converts ORB-SLAM3's optical-convention world
  // into ROS base_link convention:
  //   T_odom_base = T_base_cam * T_wc * T_base_cam^{-1}
  //
  // At t=0: T_wc = I  →  T_odom_base = I  (base_link starts at odom origin) ✓
  // Camera forward (optical z) → base_link forward (x) ✓
  const Eigen::Isometry3d T_odom_base =
    T_base_cam_ * pose_map_camera * T_cam_base_;

  if (publish_odom_to_base_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = stamp;
    tf_msg.header.frame_id = invert_odom_to_base_tf_ ? base_frame_ : odom_frame_;
    tf_msg.child_frame_id  = invert_odom_to_base_tf_ ? odom_frame_ : base_frame_;

    const Eigen::Isometry3d T = invert_odom_to_base_tf_
      ? T_odom_base.inverse()
      : T_odom_base;

    tf_msg.transform = conversion::IsometryToRosTf(T);
    tf_broadcaster_->sendTransform(tf_msg);
  }

  if (publish_map_to_odom_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id  = odom_frame_;
    tf_msg.transform       = conversion::IsometryToRosTf(Eigen::Isometry3d::Identity());
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void VisualSlamNode::PublishOdometry(
  const rclcpp::Time & stamp,
  const TrackingResult & result)
{
  // Apply the same optical→ROS coordinate frame conversion as PublishTf
  const Eigen::Isometry3d pose =
    T_base_cam_ * result.pose_map_camera * T_cam_base_;

  // PoseStamped
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = stamp;
    ps.header.frame_id = odom_frame_;
    ps.pose = conversion::IsometryToRosPose(pose);
    vo_pose_pub_->publish(ps);

    // Accumulate path
    vo_path_.push_back(ps);
    if (vo_path_.size() > path_max_size_) {
      vo_path_.erase(vo_path_.begin());
    }
    nav_msgs::msg::Path path_msg;
    path_msg.header = ps.header;
    path_msg.poses  = vo_path_;
    vo_path_pub_->publish(path_msg);
  }

  // PoseWithCovarianceStamped
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pcs;
    pcs.header.stamp    = stamp;
    pcs.header.frame_id = odom_frame_;
    pcs.pose.pose = conversion::IsometryToRosPose(pose);
    // Flatten 6×6 covariance (row-major)
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        pcs.pose.covariance[r * 6 + c] = result.covariance(r, c);
      }
    }
    vo_pose_cov_pub_->publish(pcs);
  }

  // Odometry (with twist from finite difference)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;
    odom.pose.pose = conversion::IsometryToRosPose(pose);

    const int64_t ts_ns = stamp.nanoseconds();
    if (prev_pose_.has_value()) {
      const double dt =
        (ts_ns - prev_pose_->first) * 1e-9;
      if (dt > 0.0) {
        const Eigen::Vector3d v =
          (pose.translation() - prev_pose_->second.translation()) / dt;
        odom.twist.twist.linear.x = v.x();
        odom.twist.twist.linear.y = v.y();
        odom.twist.twist.linear.z = v.z();
      }
    }
    prev_pose_ = {ts_ns, pose};
    odometry_pub_->publish(odom);
  }
}

void VisualSlamNode::PublishStatus(
  const rclcpp::Time & stamp,
  const TrackingResult & result)
{
  isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus status;
  status.header.stamp    = stamp;
  status.header.frame_id = odom_frame_;
  status.vo_state        = static_cast<uint8_t>(result.state);
  status.track_execution_time      = result.track_time_s;
  status.node_callback_execution_time = result.total_time_s;
  status_pub_->publish(status);
}

void VisualSlamNode::PublishVisualization(
  const rclcpp::Time & stamp,
  const TrackingResult & result)
{
  if (enable_observations_view_ && !result.observations.empty()) {
    observations_pub_->publish(
      EigenVectorsToPointCloud2(result.observations, odom_frame_, stamp));
  }
  if (enable_landmarks_view_ && !result.map_points.empty()) {
    landmarks_pub_->publish(
      EigenVectorsToPointCloud2(result.map_points, map_frame_, stamp));
  }
}

sensor_msgs::msg::PointCloud2 VisualSlamNode::EigenVectorsToPointCloud2(
  const std::vector<Eigen::Vector3f> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 pc;
  pc.header.stamp    = stamp;
  pc.header.frame_id = frame_id;
  pc.height = 1;
  pc.width  = static_cast<uint32_t>(points.size());
  pc.is_dense = false;
  pc.is_bigendian = false;

  // Fields: x, y, z (float32)
  pc.fields.resize(3);
  for (int i = 0; i < 3; ++i) {
    pc.fields[i].name     = std::string(1, 'x' + i);
    pc.fields[i].offset   = static_cast<uint32_t>(i * 4);
    pc.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc.fields[i].count    = 1;
  }
  pc.point_step = 12;
  pc.row_step   = pc.point_step * pc.width;
  pc.data.resize(pc.row_step);

  for (size_t i = 0; i < points.size(); ++i) {
    std::memcpy(&pc.data[i * 12 + 0], &points[i].x(), 4);
    std::memcpy(&pc.data[i * 12 + 4], &points[i].y(), 4);
    std::memcpy(&pc.data[i * 12 + 8], &points[i].z(), 4);
  }
  return pc;
}

// ---------------------------------------------------------------------------
// Service handlers
// ---------------------------------------------------------------------------

void VisualSlamNode::HandleReset(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset::Request> /*req*/,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset::Response> res)
{
  if (slam_backend_) {
    slam_backend_->Reset();
    prev_pose_.reset();
    vo_path_.clear();
    slam_path_.clear();
    initial_camera_info_.clear();
    initial_imu_msg_.reset();
  }
  res->success = true;
}

void VisualSlamNode::HandleGetAllPoses(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses::Response> res)
{
  res->success = false;
  if (!slam_backend_ || !slam_backend_->IsInitialized()) { return; }

  const auto kf_poses = slam_backend_->GetAllKeyFramePoses(req->max_count);
  res->poses.reserve(kf_poses.size());
  for (const auto & kp : kf_poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = map_frame_;
    ps.header.stamp = rclcpp::Time(kp.timestamp_ns);
    ps.pose = conversion::IsometryToRosPose(kp.pose_map_camera);
    res->poses.push_back(ps);
  }
  res->success = true;
}

void VisualSlamNode::HandleSetSlamPose(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetSlamPose::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetSlamPose::Response> res)
{
  res->success = false;
  if (!slam_backend_ || !slam_backend_->IsInitialized()) { return; }
  const Eigen::Isometry3d pose = conversion::RosPoseToIsometry(req->pose);
  res->success = slam_backend_->SetSlamPose(pose);
}

void VisualSlamNode::HandleSaveMap(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Response> res)
{
  res->success = false;
  if (!slam_backend_ || !slam_backend_->IsInitialized()) { return; }
  res->success = slam_backend_->SaveMap(req->file_path);
}

void VisualSlamNode::HandleLoadMap(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Response> res)
{
  res->success = false;
  if (!slam_backend_ || !slam_backend_->IsInitialized()) { return; }
  res->success = slam_backend_->LoadMap(req->file_path);
}

void VisualSlamNode::HandleLocalizeInMap(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap::Response> res)
{
  res->success = false;
  if (!slam_backend_ || !slam_backend_->IsInitialized()) { return; }

  const Eigen::Isometry3d hint = conversion::RosPoseToIsometry(req->pose_hint);
  const auto result = slam_backend_->LocalizeInMap(req->map_folder_path, hint);
  res->success = result.has_value();
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

Eigen::Isometry3d VisualSlamNode::LookupTransform(
  const std::string & target, const std::string & source)
{
  const auto tf_stamped = tf_buffer_->lookupTransform(
    target, source, tf2::TimePointZero,
    tf2::durationFromSec(1.0));
  return conversion::RosTfToIsometry(tf_stamped.transform);
}

}  // namespace isaac_ros::visual_slam_orb

// ── Component registration ───────────────────────────────────────────────
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros::visual_slam_orb::VisualSlamNode)
