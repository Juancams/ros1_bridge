// Copyright 2015 Open Source Robotics Foundation, Inc.
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

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"


ros::Publisher pub;

void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr ros2_msg)
{
  if (pub.getNumSubscribers() == 0)
    return;

  trajectory_msgs::JointTrajectory ros1_msg;
  ros1_msg.header.frame_id = ros2_msg->header.frame_id;
  ros1_msg.header.stamp.sec = ros2_msg->header.stamp.sec;
  ros1_msg.header.stamp.nsec = ros2_msg->header.stamp.nanosec;
  ros1_msg.joint_names = std::move(ros2_msg->joint_names);

  ros1_msg.points.resize(ros2_msg->points.size());
  for (size_t i = 0; i < ros2_msg->points.size(); i++) {
    ros1_msg.points[i].positions = std::move(ros2_msg->points[i].positions);
    ros1_msg.points[i].velocities = std::move(ros2_msg->points[i].velocities);
    ros1_msg.points[i].accelerations = std::move(ros2_msg->points[i].accelerations);
    ros1_msg.points[i].effort = std::move(ros2_msg->points[i].effort);
    ros1_msg.points[i].time_from_start.sec = ros2_msg->points[i].time_from_start.sec;
    ros1_msg.points[i].time_from_start.nsec = ros2_msg->points[i].time_from_start.nanosec;
  }

  pub.publish(ros1_msg);
}

int main(int argc, char * argv[])
{
  // ROS 2 node and subscriber
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("jointtrajectory_2_to_1");
  auto sub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "input", 100, jointTrajectoryCallback);


  // ROS 1 node and publisher
  ros::init(argc, argv, "jointtrajectory_2_to_1");
  ros::NodeHandle n;
  pub = n.advertise<trajectory_msgs::JointTrajectory>("output", 100);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
