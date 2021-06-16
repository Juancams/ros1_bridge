// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <utility>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "device_control_msgs/DeviceInfo.h"
#ifdef __clang__s
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "device_control_msgs/msg/device_info.hpp"


rclcpp::Publisher<device_control_msgs::msg::DeviceInfo>::SharedPtr pub;


void DeviceInfoCallback(boost::shared_ptr<device_control_msgs::DeviceInfo> ros1_msg)
{
  auto ros2_msg = std::make_unique<device_control_msgs::msg::DeviceInfo>();

  ros2_msg->system_source = ros1_msg->system_source;
  ros2_msg->ros_version_source = ros1_msg->ros_version_source;
  ros2_msg->topics = std::move(ros1_msg->topics);

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mocap_info_1_to_2");
  pub = node->create_publisher<device_control_msgs::msg::DeviceInfo>("device_environment",
    rclcpp::QoS(1000).reliable().transient_local().keep_all());

  // ROS 1 node and subscriber
  ros::init(argc, argv, "mocap_info_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("device_environment", 1000, DeviceInfoCallback);

  ros::spin();

  rclcpp::shutdown();

  return 0;
}