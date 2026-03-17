# Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Unified launch file for ORB-SLAM3 with Intel RealSense D456.
# Supports all sensor modes via the 'mode' argument.
#
# GPU preprocessing via isaac_ros_image_proc (optional, enabled by default):
#   - ResizeNode: GPU-accelerated resize (stereo/mono: 848x480 → 640x480)
#   - ImageFormatConverterNode: GPU color conversion (RGBD: RGB8 → mono8)
#
# Usage:
#   ros2 launch isaac_ros_visual_slam_orb \
#       isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=stereo
#
#   # Disable GPU preprocessing (use CPU cv_bridge fallback)
#   ros2 launch isaac_ros_visual_slam_orb \
#       isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=stereo gpu_preprocess:=false

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    """Resolve launch arguments and build the appropriate node graph."""

    pkg_share = get_package_share_directory('isaac_ros_visual_slam_orb')
    mode = LaunchConfiguration('mode').perform(context)
    gpu_preprocess = LaunchConfiguration('gpu_preprocess').perform(context) == 'true'

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
        sensor_mode_str = mode

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
        rs_params['unite_imu_method']  = 2
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

    # ── GPU preprocessing nodes (isaac_ros_image_proc) ────────────────────
    # These use NITROS and run in a separate container from the SLAM node.
    preproc_nodes = []
    nodes_to_launch = [realsense_node]

    # Note: ResizeNode is NOT used for stereo/mono IR streams.
    # RealSense outputs 640x480 natively via depth_module.infra_profile.
    # GPU resize would cause aspect ratio mismatch (848:480≠640:480 → fx≠fy).

    if gpu_preprocess and use_rgbd:
        # ImageFormatConverterNode: RGB8 → mono8 on GPU
        preproc_nodes.append(ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
            name='color_to_mono',
            parameters=[{
                'encoding_desired': 'mono8',
                'image_width': 640,
                'image_height': 480,
            }],
            remappings=[
                ('image_raw', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
                ('image', '/preproc/color/image_mono'),
            ],
        ))

    # Launch preprocessing container if nodes exist
    if preproc_nodes:
        preproc_container = ComposableNodeContainer(
            name='preproc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=preproc_nodes,
            output='screen',
        )
        nodes_to_launch.append(preproc_container)

    # ── Topic remappings (GPU preprocessed or raw) ────────────────────────
    remappings = []
    if use_stereo:
        # Stereo IR: direct from RealSense (already 640x480 via infra_profile)
        remappings = [
            ('/visual_slam/image_0',       '/camera/infra1/image_rect_raw'),
            ('/visual_slam/camera_info_0', '/camera/infra1/camera_info'),
            ('/visual_slam/image_1',       '/camera/infra2/image_rect_raw'),
            ('/visual_slam/camera_info_1', '/camera/infra2/camera_info'),
        ]
    elif use_rgbd:
        if gpu_preprocess:
            # RGBD: GPU color conversion (RGB8 → mono8) + raw depth
            remappings = [
                ('/visual_slam/image_0',       '/preproc/color/image_mono'),
                ('/visual_slam/camera_info_0', '/camera/color/camera_info'),
                ('/visual_slam/depth',         '/camera/aligned_depth_to_color/image_raw'),
            ]
        else:
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

    # ── ORB-SLAM3 node parameters ─────────────────────────────────────────
    vslam_params = {
        'sensor_mode':              sensor_mode_str,
        'num_cameras':              num_cameras,
        'sync_matching_threshold_ms': 5.0,
        'min_num_images':           1,
        'image_buffer_size':        10,
        'imu_buffer_size':          200,
        'image_jitter_threshold_ms': 34.0,
        'imu_jitter_threshold_ms':  10.0,
        'rectified_images':         LaunchConfiguration('rectified_images'),
        'enable_image_denoising':   LaunchConfiguration('enable_image_denoising'),
        'orb_vocab_path':           LaunchConfiguration('orb_vocab_path'),
        'orb_settings_path':        os.path.join(pkg_share, 'config', 'realsense_d456.yaml'),
        'orb_num_features':         LaunchConfiguration('orb_num_features'),
        'orb_scale_factor':         LaunchConfiguration('orb_scale_factor'),
        'orb_num_levels':           LaunchConfiguration('orb_num_levels'),
        'map_frame':                'map',
        'odom_frame':               'odom',
        'base_frame':               'camera_link',
        'publish_map_to_odom_tf':   True,
        'publish_odom_to_base_tf':  True,
        'enable_slam_visualization': LaunchConfiguration('enable_slam_visualization'),
        'enable_observations_view': LaunchConfiguration('enable_observations_view'),
        'enable_landmarks_view':    LaunchConfiguration('enable_landmarks_view'),
        'override_publishing_stamp': LaunchConfiguration('override_publishing_stamp'),
        'path_max_size':            1024,
    }

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

    if use_stereo:
        vslam_params['camera_optical_frames'] = [
            'camera_infra1_optical_frame',
            'camera_infra2_optical_frame',
        ]
    elif use_rgbd:
        vslam_params['camera_optical_frames'] = [
            'camera_color_optical_frame',
        ]
    else:
        vslam_params['camera_optical_frames'] = [
            'camera_infra1_optical_frame',
        ]

    # ── VSLAM composable node container ───────────────────────────────────
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
    nodes_to_launch.append(vslam_container)

    return nodes_to_launch


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('mode',
            default_value='stereo',
            choices=['mono', 'mono-imu', 'stereo', 'stereo-imu', 'rgbd', 'rgbd-imu'],
            description='Sensor mode'),
        DeclareLaunchArgument('gpu_preprocess',
            default_value='true',
            choices=['true', 'false'],
            description='Enable GPU image preprocessing via isaac_ros_image_proc'),

        DeclareLaunchArgument('rectified_images',       default_value='true'),
        DeclareLaunchArgument('enable_image_denoising',  default_value='false'),
        DeclareLaunchArgument('enable_slam_visualization', default_value='true'),
        DeclareLaunchArgument('enable_observations_view',  default_value='true'),
        DeclareLaunchArgument('enable_landmarks_view',     default_value='true'),
        DeclareLaunchArgument('override_publishing_stamp', default_value='false'),

        DeclareLaunchArgument('orb_num_features', default_value='1000'),
        DeclareLaunchArgument('orb_scale_factor',  default_value='1.2'),
        DeclareLaunchArgument('orb_num_levels',    default_value='8'),
        DeclareLaunchArgument('orb_vocab_path',
            default_value='/opt/orb_slam3/Vocabulary/ORBvoc.txt'),

        DeclareLaunchArgument('gyro_noise_density',   default_value='0.000244'),
        DeclareLaunchArgument('gyro_random_walk',     default_value='0.000019393'),
        DeclareLaunchArgument('accel_noise_density',  default_value='0.001862'),
        DeclareLaunchArgument('accel_random_walk',    default_value='0.003'),
        DeclareLaunchArgument('calibration_frequency', default_value='200.0'),

        OpaqueFunction(function=_launch_setup),
    ])
