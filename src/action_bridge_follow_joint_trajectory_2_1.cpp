// Copyright 2019 Fraunhofer IPA
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

#include <ros1_bridge/action_bridge_2_1.hpp>

// include ROS 1
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <control_msgs/FollowJointTrajectoryAction.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include "control_msgs/action/follow_joint_trajectory.hpp"

template <typename T1, typename T2>
static void copy_point(const T1 &pt1, T2 &pt2)
{
  pt2.positions = pt1.positions;
  pt2.velocities = pt1.velocities;
  pt2.accelerations = pt1.accelerations;
}

template <typename T1, typename T2>
static void copy_tolerance(const T1 &tolerance1, T2 &tolerance2)
{
  tolerance2.name = tolerance1.name;
  tolerance2.position = tolerance1.position;
  tolerance2.velocity = tolerance1.velocity;
  tolerance2.acceleration = tolerance1.acceleration;
}

template <typename T1, typename T2>
static void copy_tolerances(const T1 &t1, T2 &t2)
{
  const size_t num = t1.size();
  t2.resize(num);
  for (size_t i = 0; i < num; ++i)
  {
    copy_tolerance(t1[i], t2[i]);
  }
}

static void
copy_duration_2_to_1(const builtin_interfaces::msg::Duration &duration2,
                     ros::Duration &duration1)
{
  duration1.sec = duration2.sec;
  duration1.nsec = duration2.nanosec;
}

using FollowJointTrajectoryActionBridge =
    ActionBridge_2_1<control_msgs::FollowJointTrajectoryAction,
                     control_msgs::action::FollowJointTrajectory>;

template <>
void FollowJointTrajectoryActionBridge::translate_goal_2_to_1(
    const ROS2Goal &goal2, ROS1Goal &goal1)
{
  goal1.trajectory.header.frame_id = goal2.trajectory.header.frame_id;
  goal1.trajectory.joint_names = goal2.trajectory.joint_names;
  const size_t num = goal2.trajectory.points.size();
  goal1.trajectory.points.resize(num);

  for (size_t i = 0; i < num; ++i)
  {
    copy_point(goal2.trajectory.points[i], goal1.trajectory.points[i]);
    copy_duration_2_to_1(goal2.trajectory.points[i].time_from_start,
                         goal1.trajectory.points[i].time_from_start);
  }

  copy_tolerances(goal2.path_tolerance, goal1.path_tolerance);
  copy_tolerances(goal2.goal_tolerance, goal1.goal_tolerance);

  copy_duration_2_to_1(goal2.goal_time_tolerance, goal1.goal_time_tolerance);
}

template <>
void FollowJointTrajectoryActionBridge::translate_result_1_to_2(
    ROS2Result &result2, const ROS1Result &result1)
{
  result2.error_code = result1.error_code;
  result2.error_string = result1.error_string;
}

template <>
void FollowJointTrajectoryActionBridge::translate_feedback_1_to_2(
    ROS2Feedback &feedback2, const ROS1Feedback &feedback1)
{
  feedback2.joint_names = feedback1.joint_names;
  copy_point(feedback1.desired, feedback2.desired);
  copy_point(feedback1.actual, feedback2.actual);
  copy_point(feedback1.error, feedback2.error);
}

int main(int argc, char *argv[])
{
    std::string action_name_;
    action_name_ = "/arm_controller/follow_joint_trajectory";

  return FollowJointTrajectoryActionBridge::main(action_name_,
                                                 argc, argv);
}
