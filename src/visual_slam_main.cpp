// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include "isaac_ros_visual_slam_orb/visual_slam_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<isaac_ros::visual_slam_orb::VisualSlamNode>());
  rclcpp::shutdown();
  return 0;
}
