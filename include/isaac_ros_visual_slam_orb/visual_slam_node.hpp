// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// Re-use all ROS interface types from the existing visual_slam_interfaces
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Same service definitions as isaac_ros_visual_slam (drop-in compatible)
#include <isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/file_path.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/get_all_poses.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/localize_in_map.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/reset.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/set_slam_pose.hpp>

#include "isaac_ros_visual_slam_orb/slam_backend.hpp"
#include "isaac_ros_visual_slam_orb/slam_types.hpp"

// Re-use synchronisation utilities from isaac_ros_visual_slam
#include <isaac_common/messaging/message_stream_synchronizer.hpp>

namespace isaac_ros::visual_slam_orb
{

// ---------------------------------------------------------------------------
// VisualSlamNode
//
// ROS2 node that exposes the same topics, services, and TF tree as
// isaac_ros_visual_slam (cuVSLAM), but uses ISlamBackend (ORB-SLAM3) as
// the underlying SLAM engine.
//
// Drop-in replacement: all topic names, service names, and parameter names
// are identical to the original node so existing launch files, rviz configs,
// and downstream nodes work without modification.
// ---------------------------------------------------------------------------
class VisualSlamNode : public rclcpp::Node
{
public:
  explicit VisualSlamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VisualSlamNode() override;

private:
  // ── Parameters (mirrors isaac_ros_visual_slam parameter names) ──────────
  const uint32_t num_cameras_;
  const double   sync_matching_threshold_ms_;
  const uint32_t min_num_images_;
  const bool     enable_imu_fusion_;
  const bool     rectified_images_;
  const bool     enable_image_denoising_;
  const bool     enable_localization_n_mapping_;
  const bool     enable_ground_constraint_in_odometry_;
  const bool     publish_map_to_odom_tf_;
  const bool     publish_odom_to_base_tf_;
  const bool     invert_map_to_odom_tf_;
  const bool     invert_odom_to_base_tf_;
  const bool     enable_slam_visualization_;
  const bool     enable_observations_view_;
  const bool     enable_landmarks_view_;
  const bool     override_publishing_stamp_;
  const uint32_t path_max_size_;
  const uint32_t image_buffer_size_;
  const uint32_t imu_buffer_size_;
  const double   image_jitter_threshold_ms_;
  const double   imu_jitter_threshold_ms_;

  const std::string map_frame_;
  const std::string odom_frame_;
  const std::string base_frame_;
  const std::vector<std::string> camera_optical_frames_;
  const std::string imu_frame_;

  // Sensor configuration
  const std::string sensor_mode_str_;
  const std::string depth_topic_;

  enum class SensorMode {
    kMono,
    kMonoImu,
    kStereo,
    kStereoImu,
    kRgbd,
    kRgbdImu
  };
  SensorMode sensor_mode_{SensorMode::kStereo};


  // ORB-SLAM3 specific parameters
  const std::string orb_vocab_path_;
  const std::string orb_settings_path_;
  const int         orb_num_features_;
  const float       orb_scale_factor_;
  const int         orb_num_levels_;

  // IMU noise parameters (same names as isaac_ros_visual_slam)
  const double gyro_noise_density_;
  const double gyro_random_walk_;
  const double accel_noise_density_;
  const double accel_random_walk_;
  const double calibration_frequency_;

  // ── SLAM backend ────────────────────────────────────────────────────────
  std::unique_ptr<ISlamBackend> slam_backend_;

  // ── Subscribers ─────────────────────────────────────────────────────────
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>      image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr                   depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                     imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;

  // ── Publishers ──────────────────────────────────────────────────────────
  rclcpp::Publisher<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::SharedPtr
    status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr             vo_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    vo_pose_cov_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                     odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                         vo_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                         slam_path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr       diagnostics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr               observations_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr               landmarks_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr               pose_graph_pub_;

  // ── Services ────────────────────────────────────────────────────────────
  rclcpp::Service<isaac_ros_visual_slam_interfaces::srv::Reset>::SharedPtr        reset_srv_;
  rclcpp::Service<isaac_ros_visual_slam_interfaces::srv::GetAllPoses>::SharedPtr  get_poses_srv_;
  rclcpp::Service<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>::SharedPtr  set_pose_srv_;
  rclcpp::Service<isaac_ros_visual_slam_interfaces::srv::FilePath>::SharedPtr     save_map_srv_;
  rclcpp::Service<isaac_ros_visual_slam_interfaces::srv::FilePath>::SharedPtr     load_map_srv_;
  rclcpp::Service<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap>::SharedPtr localize_srv_;

  // ── TF ──────────────────────────────────────────────────────────────────
  std::unique_ptr<tf2_ros::Buffer>             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener>  tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ── Synchroniser (multi-camera image sync) ──────────────────────────────
  using Synchronizer =
    nvidia::isaac_common::messaging::MessageStreamSynchronizer<sensor_msgs::msg::Image>;
  // Heap-allocated because MessageStreamSynchronizer has no move-assignment
  std::unique_ptr<Synchronizer> sync_;

  // ── Internal state ──────────────────────────────────────────────────────
  // number of image streams we expect (monocular=1, stereo=2), used by sync
  uint32_t effective_num_cameras_ = 0;
  bool rgbd_mode_ = false;

  std::map<int, std::optional<sensor_msgs::msg::CameraInfo::ConstSharedPtr>>
    initial_camera_info_;
  std::optional<sensor_msgs::msg::Imu::ConstSharedPtr> initial_imu_msg_;

  // IMU message queue between frames
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> pending_imu_;
  std::mutex imu_mutex_;

  // Pose trail for path visualisation
  std::vector<geometry_msgs::msg::PoseStamped> vo_path_;
  std::vector<geometry_msgs::msg::PoseStamped> slam_path_;

  // Previous published odom pose for velocity calculation
  std::optional<std::pair<int64_t, Eigen::Isometry3d>> prev_pose_;

  // Static transform: base_frame → camera_optical_frame (from TF tree).
  // Used to convert ORB-SLAM3 poses (optical convention) to ROS convention.
  // T_odom_base = T_base_cam * T_wc * T_base_cam^{-1}
  Eigen::Isometry3d T_base_cam_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_cam_base_{Eigen::Isometry3d::Identity()};

  // ── Callbacks ───────────────────────────────────────────────────────────
  void OnImage(int index, const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void OnDepth(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void OnCameraInfo(int index, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void OnInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);

  void OnSyncedImages(
    int64_t timestamp_ns,
    const std::vector<std::pair<int, sensor_msgs::msg::Image>> & images);

  void TryInitialize();
  bool IsReadyForInitialization() const;

  // ── Publishing helpers ───────────────────────────────────────────────────
  void PublishAll(
    const rclcpp::Time & stamp,
    const TrackingResult & result);

  void PublishTf(
    const rclcpp::Time & stamp,
    const Eigen::Isometry3d & pose_map_camera);

  void PublishOdometry(
    const rclcpp::Time & stamp,
    const TrackingResult & result);

  void PublishStatus(
    const rclcpp::Time & stamp,
    const TrackingResult & result);

  void PublishVisualization(
    const rclcpp::Time & stamp,
    const TrackingResult & result);

  // ── Service handlers ────────────────────────────────────────────────────
  void HandleReset(
    const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset::Request> req,
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset::Response> res);

  void HandleGetAllPoses(
    const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses::Request> req,
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses::Response> res);

  void HandleSetSlamPose(
    const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetSlamPose::Request> req,
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetSlamPose::Response> res);

  void HandleSaveMap(
    const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Request> req,
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Response> res);

  void HandleLoadMap(
    const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Request> req,
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Response> res);

  void HandleLocalizeInMap(
    const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap::Request> req,
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap::Response> res);

  // ── Utilities ────────────────────────────────────────────────────────────
  Eigen::Isometry3d LookupTransform(
    const std::string & target, const std::string & source);

  sensor_msgs::msg::PointCloud2 EigenVectorsToPointCloud2(
    const std::vector<Eigen::Vector3f> & points,
    const std::string & frame_id,
    const rclcpp::Time & stamp);

  rclcpp::Time EffectiveStamp(const rclcpp::Time & stamp) const;
};

}  // namespace isaac_ros::visual_slam_orb
