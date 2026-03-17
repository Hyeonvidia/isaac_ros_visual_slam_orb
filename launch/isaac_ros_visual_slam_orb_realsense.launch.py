# Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Launch file for ORB-SLAM3 visual SLAM with Intel RealSense D456
# on Jetson AGX Orin inside the isaac_ros Docker container.
#
# Topic remapping is identical to isaac_ros_visual_slam_realsense.launch.py
# so RViz configs and downstream nodes work unchanged.
#
# Usage:
#   ros2 launch isaac_ros_visual_slam_orb \
#       isaac_ros_visual_slam_orb_realsense.launch.py \
#       enable_imu_fusion:=true

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    pkg_share = get_package_share_directory('isaac_ros_visual_slam_orb')

    # ── Launch arguments (identical names to isaac_ros_visual_slam) ────────
    args = [
        DeclareLaunchArgument('enable_imu_fusion',
                              default_value='true',
                              description='Fuse D456 IMU with stereo visual odometry'),
        DeclareLaunchArgument('rectified_images',
                              default_value='true',
                              description='Images are already rectified by RealSense SDK'),
        DeclareLaunchArgument('enable_image_denoising',
                              default_value='false',
                              description='Apply Gaussian blur before feature extraction'),
        DeclareLaunchArgument('enable_slam_visualization',
                              default_value='true'),
        DeclareLaunchArgument('enable_observations_view',
                              default_value='true'),
        DeclareLaunchArgument('enable_landmarks_view',
                              default_value='true'),
        DeclareLaunchArgument('orb_num_features',
                              default_value='1000',
                              description='Number of ORB features per frame'),
        DeclareLaunchArgument('orb_scale_factor',
                              default_value='1.2'),
        DeclareLaunchArgument('orb_num_levels',
                              default_value='8'),
        DeclareLaunchArgument('orb_vocab_path',
                              default_value='/opt/orb_slam3/Vocabulary/ORBvoc.txt',
                              description='Path to ORB-SLAM3 vocabulary file'),
        # D456 IMU noise parameters (BMI055)
        DeclareLaunchArgument('gyro_noise_density',  default_value='0.000244'),
        DeclareLaunchArgument('gyro_random_walk',    default_value='0.000019393'),
        DeclareLaunchArgument('accel_noise_density', default_value='0.001862'),
        DeclareLaunchArgument('accel_random_walk',   default_value='0.003'),
        DeclareLaunchArgument('calibration_frequency', default_value='200.0'),
        DeclareLaunchArgument('override_publishing_stamp',
                              default_value='false',
                              description='Set true when playing rosbags'),
    ]

    # ── RealSense D456 node ───────────────────────────────────────────────
    # Uses realsense2_camera package (installed in isaac_ros Docker image)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            # Stereo infra (IR) streams — best for SLAM (no lighting sensitivity)
            'enable_infra1':       True,
            'enable_infra2':       True,
            'enable_color':        False,
            'enable_depth':        False,
            # Resolution and FPS — D456 uses depth_module profile for IR streams
            'depth_module.infra_profile': '640x480x30',
            # IMU
            'enable_gyro':         True,
            'enable_accel':        True,
            'gyro_fps':            200,
            'accel_fps':           100,
            'unite_imu_method':    2,     # 2 = linear_interpolation
            # Ensure images are rectified
            'enable_sync':         True,
            # Disable IR emitter (important for stereo matching in ambient light)
            'emitter_enabled':     0,
        }],
        output='screen',
    )

    # ── Image format conversion (if color images used instead of IR) ────────
    # D456 IR images come as 8-bit mono — no conversion needed for ORB-SLAM3.
    # If you switch to color streams, add image_proc nodes here.

    # ── ORB-SLAM3 visual SLAM node ────────────────────────────────────────
    vslam_node = ComposableNode(
        package='isaac_ros_visual_slam_orb',
        plugin='isaac_ros::visual_slam_orb::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            # Camera / sync
            'num_cameras':              2,
            'sync_matching_threshold_ms': 5.0,
            'min_num_images':           1,
            'image_buffer_size':        10,
            'imu_buffer_size':          100,
            'image_jitter_threshold_ms': 34.0,
            'imu_jitter_threshold_ms':  10.0,

            # Image processing
            'rectified_images':         LaunchConfiguration('rectified_images'),
            'enable_image_denoising':   LaunchConfiguration('enable_image_denoising'),

            # IMU
            'enable_imu_fusion':        LaunchConfiguration('enable_imu_fusion'),
            'imu_frame':                'camera_gyro_optical_frame',
            'gyro_noise_density':       LaunchConfiguration('gyro_noise_density'),
            'gyro_random_walk':         LaunchConfiguration('gyro_random_walk'),
            'accel_noise_density':      LaunchConfiguration('accel_noise_density'),
            'accel_random_walk':        LaunchConfiguration('accel_random_walk'),
            'calibration_frequency':    LaunchConfiguration('calibration_frequency'),

            # ORB feature extractor
            'orb_vocab_path':           LaunchConfiguration('orb_vocab_path'),
            'orb_settings_path':        os.path.join(pkg_share,
                                            'config', 'realsense_d456.yaml'),
            'orb_num_features':         LaunchConfiguration('orb_num_features'),
            'orb_scale_factor':         LaunchConfiguration('orb_scale_factor'),
            'orb_num_levels':           LaunchConfiguration('orb_num_levels'),

            # Frame IDs
            'map_frame':                'map',
            'odom_frame':               'odom',
            'base_frame':               'camera_link',
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],

            # Output
            'publish_map_to_odom_tf':   True,
            'publish_odom_to_base_tf':  True,
            'enable_slam_visualization': LaunchConfiguration('enable_slam_visualization'),
            'enable_observations_view': LaunchConfiguration('enable_observations_view'),
            'enable_landmarks_view':    LaunchConfiguration('enable_landmarks_view'),
            'override_publishing_stamp': LaunchConfiguration('override_publishing_stamp'),
            'path_max_size':            1024,
        }],
        remappings=[
            # Camera 0 (left IR / infra1)
            ('/visual_slam/image_0',       '/camera/infra1/image_rect_raw'),
            ('/visual_slam/camera_info_0', '/camera/infra1/camera_info'),
            # Camera 1 (right IR / infra2)
            ('/visual_slam/image_1',       '/camera/infra2/image_rect_raw'),
            ('/visual_slam/camera_info_1', '/camera/infra2/camera_info'),
            # IMU (combined gyro + accel from realsense2_camera)
            ('/visual_slam/imu',           '/camera/imu'),
        ],
    )

    vslam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[vslam_node],
        output='screen',
    )

    return launch.LaunchDescription(args + [realsense_node, vslam_container])
