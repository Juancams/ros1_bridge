// Copyright 2023 Juan Carlos Manzanares Serrano
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
#include "sensor_msgs/CameraInfo.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub;


void cameraCallback(boost::shared_ptr<sensor_msgs::CameraInfo> ros1_msg)
{
  if (pub->get_subscription_count() == 0)
    return;

  auto ros2_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();

  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  ros2_msg->header.stamp = rclcpp::Time(ros1_msg->header.stamp.toNSec());

  ros2_msg->height = ros1_msg->height;
  ros2_msg->width = ros1_msg->width;
  ros2_msg->distortion_model = ros1_msg->distortion_model;
  ros2_msg->d = std::move(ros1_msg->D);
  ros2_msg->k = std::move(ros1_msg->K);
  ros2_msg->r = std::move(ros1_msg->R);
  ros2_msg->p = std::move(ros1_msg->P);
  ros2_msg->binning_x = ros1_msg->binning_x;
  ros2_msg->binning_y = ros1_msg->binning_y;
  ros2_msg->roi = std::move(ros1_msg->roi);

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("camera_info_1_to_2");
  pub = node->create_publisher<sensor_msgs::msg::Image>("/xtion/rgb/camera_info", 100);

  // ROS 1 node and subscriber
  ros::init(argc, argv, "camera_info_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/xtion/rgb/camera_info", 100, cameraCallback);

  ros::spin();

  rclcpp::shutdown();

  return 0;
}
