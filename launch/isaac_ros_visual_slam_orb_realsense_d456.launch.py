# Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Unified launch file for ORB-SLAM3 with Intel RealSense D456.
# Supports all ORB-SLAM3 sensor modes via the 'mode' argument:
#
#   mono              — Monocular (left IR only)
#   mono-inertial     — Monocular-Inertial (left IR + IMU)
#   stereo            — Stereo (left + right IR)
#   stereo-inertial   — Stereo-Inertial (left + right IR + IMU)
#   rgb-d             — RGB-D (color + depth)
#
# Aliases accepted: mono-imu, stereo-imu, rgbd, rgbd-imu
#
# Usage:
#   ros2 launch isaac_ros_visual_slam_orb \
#       isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=stereo
#
#   ros2 launch isaac_ros_visual_slam_orb \
#       isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=stereo-inertial
#
#   ros2 launch isaac_ros_visual_slam_orb \
#       isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=mono
#
#   ros2 launch isaac_ros_visual_slam_orb \
#       isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=rgb-d

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    """Resolve launch arguments and build the appropriate node graph."""

    pkg_share = get_package_share_directory('isaac_ros_visual_slam_orb')
    mode_raw = LaunchConfiguration('mode').perform(context)

    # ── Normalise mode aliases ────────────────────────────────────────────
    _ALIASES = {
        'mono-inertial': 'mono-imu',
        'stereo-inertial': 'stereo-imu',
        'rgb-d': 'rgbd',
    }
    mode = _ALIASES.get(mode_raw, mode_raw)

    # ── Derive flags from mode ────────────────────────────────────────────
    use_stereo = mode in ('stereo', 'stereo-imu')
    use_imu    = mode in ('stereo-imu', 'mono-imu', 'rgbd-imu')
    use_rgbd   = mode in ('rgbd', 'rgbd-imu')
    use_mono   = mode in ('mono', 'mono-imu')

    if use_stereo or use_mono:
        sensor_mode_str = 'stereo' if use_stereo else 'mono'
        if use_imu:
            sensor_mode_str += '-imu'
    elif use_rgbd:
        sensor_mode_str = 'rgbd-imu' if use_imu else 'rgbd'
    else:
        sensor_mode_str = mode  # pass through

    num_cameras = 2 if use_stereo else 1

    # ── RealSense D456 camera node ────────────────────────────────────────
    rs_params = {
        'enable_infra1':  True,
        'enable_infra2':  use_stereo,
        'enable_color':   use_rgbd,
        'enable_depth':   use_rgbd,
        'depth_module.infra_profile': '640x480x30',
        'enable_gyro':    use_imu,
        'enable_accel':   use_imu,
        'enable_sync':    True,
        'emitter_enabled': 0,
    }
    if use_imu:
        rs_params['gyro_fps']          = 200
        rs_params['accel_fps']         = 100
        rs_params['unite_imu_method']  = 2   # linear_interpolation
    if use_rgbd:
        rs_params['rgb_camera.profile'] = '640x480x30'
        rs_params['depth_module.profile'] = '640x480x30'
        rs_params['align_depth.enable'] = True

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[rs_params],
        output='screen',
    )

    # ── ORB-SLAM3 node parameters ─────────────────────────────────────────
    vslam_params = {
        # Sensor mode
        'sensor_mode':              sensor_mode_str,
        'num_cameras':              num_cameras,

        # Sync
        'sync_matching_threshold_ms': 5.0,
        'min_num_images':           1,
        'image_buffer_size':        10,
        'imu_buffer_size':          200,
        'image_jitter_threshold_ms': 34.0,
        'imu_jitter_threshold_ms':  10.0,

        # Image processing
        'rectified_images':         LaunchConfiguration('rectified_images'),
        'enable_image_denoising':   LaunchConfiguration('enable_image_denoising'),

        # ORB feature extractor
        'orb_vocab_path':           LaunchConfiguration('orb_vocab_path'),
        'orb_settings_path':        os.path.join(pkg_share, 'config', 'realsense_d456.yaml'),
        'orb_num_features':         LaunchConfiguration('orb_num_features'),
        'orb_scale_factor':         LaunchConfiguration('orb_scale_factor'),
        'orb_num_levels':           LaunchConfiguration('orb_num_levels'),

        # Frame IDs
        'map_frame':                'map',
        'odom_frame':               'odom',
        'base_frame':               'camera_link',

        # Output
        'publish_map_to_odom_tf':   True,
        'publish_odom_to_base_tf':  True,
        'enable_slam_visualization': LaunchConfiguration('enable_slam_visualization'),
        'enable_observations_view': LaunchConfiguration('enable_observations_view'),
        'enable_landmarks_view':    LaunchConfiguration('enable_landmarks_view'),
        'override_publishing_stamp': LaunchConfiguration('override_publishing_stamp'),
        'path_max_size':            1024,
    }

    # ── IMU parameters (only when using IMU) ──────────────────────────────
    if use_imu:
        vslam_params['enable_imu_fusion']    = True
        vslam_params['imu_frame']            = 'camera_gyro_optical_frame'
        vslam_params['gyro_noise_density']   = LaunchConfiguration('gyro_noise_density')
        vslam_params['gyro_random_walk']     = LaunchConfiguration('gyro_random_walk')
        vslam_params['accel_noise_density']  = LaunchConfiguration('accel_noise_density')
        vslam_params['accel_random_walk']    = LaunchConfiguration('accel_random_walk')
        vslam_params['calibration_frequency'] = LaunchConfiguration('calibration_frequency')
    else:
        vslam_params['enable_imu_fusion']    = False

    # ── Camera optical frames ─────────────────────────────────────────────
    if use_stereo:
        vslam_params['camera_optical_frames'] = [
            'camera_infra1_optical_frame',
            'camera_infra2_optical_frame',
        ]
    elif use_rgbd:
        vslam_params['camera_optical_frames'] = [
            'camera_color_optical_frame',
        ]
    else:  # mono
        vslam_params['camera_optical_frames'] = [
            'camera_infra1_optical_frame',
        ]

    # ── Topic remappings ──────────────────────────────────────────────────
    remappings = []
    if use_stereo:
        remappings = [
            ('/visual_slam/image_0',       '/camera/infra1/image_rect_raw'),
            ('/visual_slam/camera_info_0', '/camera/infra1/camera_info'),
            ('/visual_slam/image_1',       '/camera/infra2/image_rect_raw'),
            ('/visual_slam/camera_info_1', '/camera/infra2/camera_info'),
        ]
    elif use_rgbd:
        remappings = [
            ('/visual_slam/image_0',       '/camera/color/image_raw'),
            ('/visual_slam/camera_info_0', '/camera/color/camera_info'),
            ('/visual_slam/depth',         '/camera/aligned_depth_to_color/image_raw'),
        ]
    else:  # mono
        remappings = [
            ('/visual_slam/image_0',       '/camera/infra1/image_rect_raw'),
            ('/visual_slam/camera_info_0', '/camera/infra1/camera_info'),
        ]

    if use_imu:
        remappings.append(('/visual_slam/imu', '/camera/imu'))

    # ── Composable node container ─────────────────────────────────────────
    vslam_node = ComposableNode(
        package='isaac_ros_visual_slam_orb',
        plugin='isaac_ros::visual_slam_orb::VisualSlamNode',
        name='visual_slam_node',
        parameters=[vslam_params],
        remappings=remappings,
    )

    vslam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[vslam_node],
        output='screen',
    )

    # ── RViz2 (optional) ───────────────────────────────────────────────
    run_rviz = LaunchConfiguration('run_rviz').perform(context)

    nodes = [realsense_node, vslam_container]

    if run_rviz.lower() == 'true':
        if use_rgbd:
            rviz_cfg = 'rgbd.rviz'
        elif use_stereo:
            rviz_cfg = 'stereo.rviz'
        else:
            rviz_cfg = 'mono.rviz'

        rviz_config_path = os.path.join(pkg_share, 'rviz', rviz_cfg)
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        )
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return launch.LaunchDescription([
        # ── Mode selection ────────────────────────────────────────────────
        DeclareLaunchArgument('mode',
            default_value='stereo',
            choices=[
                'mono', 'mono-inertial', 'mono-imu',
                'stereo', 'stereo-inertial', 'stereo-imu',
                'rgb-d', 'rgbd', 'rgbd-imu',
            ],
            description='Sensor mode: mono, mono-inertial, stereo, stereo-inertial, rgb-d'),

        # ── RViz2 ─────────────────────────────────────────────────────────
        DeclareLaunchArgument('run_rviz', default_value='true',
            description='Launch RViz2 with sensor-mode-appropriate config'),

        # ── Common arguments ──────────────────────────────────────────────
        DeclareLaunchArgument('rectified_images',       default_value='true'),
        DeclareLaunchArgument('enable_image_denoising',  default_value='false'),
        DeclareLaunchArgument('enable_slam_visualization', default_value='true'),
        DeclareLaunchArgument('enable_observations_view',  default_value='true'),
        DeclareLaunchArgument('enable_landmarks_view',     default_value='true'),
        DeclareLaunchArgument('override_publishing_stamp', default_value='false'),

        # ── ORB extractor ─────────────────────────────────────────────────
        DeclareLaunchArgument('orb_num_features', default_value='1000'),
        DeclareLaunchArgument('orb_scale_factor',  default_value='1.2'),
        DeclareLaunchArgument('orb_num_levels',    default_value='8'),
        DeclareLaunchArgument('orb_vocab_path',
            default_value='/opt/orb_slam3/Vocabulary/ORBvoc.txt'),

        # ── D456 IMU noise (BMI055) — only used in *-imu modes ────────────
        DeclareLaunchArgument('gyro_noise_density',   default_value='0.001'),
        DeclareLaunchArgument('gyro_random_walk',     default_value='0.000001'),
        DeclareLaunchArgument('accel_noise_density',  default_value='0.01'),
        DeclareLaunchArgument('accel_random_walk',    default_value='0.0001'),
        DeclareLaunchArgument('calibration_frequency', default_value='200.0'),

        # ── Build node graph based on mode ────────────────────────────────
        OpaqueFunction(function=_launch_setup),
    ])
