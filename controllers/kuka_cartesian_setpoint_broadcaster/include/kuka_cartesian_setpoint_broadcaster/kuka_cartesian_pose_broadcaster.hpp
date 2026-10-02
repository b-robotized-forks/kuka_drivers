// Copyright 2026 KUKA Hungaria Kft.
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

#ifndef KUKA_CARTESIAN_SETPOINT_BROADCASTER__KUKA_CARTESIAN_POSE_BROADCASTER_HPP_
#define KUKA_CARTESIAN_SETPOINT_BROADCASTER__KUKA_CARTESIAN_POSE_BROADCASTER_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "kuka_cartesian_setpoint_broadcaster/visibility_control.h"

namespace kuka_controllers
{
// Broadcasts the robot's actual Cartesian pose (RIst), as opposed to CartesianSetpointBroadcaster
// which broadcasts the commanded/nominal setpoint (RSol) -- see the "cartesian_pose" vs
// "cartesian_setpoint" sensor components in kuka.ros2_control.xacro.
class CartesianPoseBroadcaster : public controller_interface::ControllerInterface
{
public:
  KUKA_CARTESIAN_SETPOINT_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  KUKA_CARTESIAN_SETPOINT_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  KUKA_CARTESIAN_SETPOINT_BROADCASTER_PUBLIC controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  KUKA_CARTESIAN_SETPOINT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_CARTESIAN_SETPOINT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_CARTESIAN_SETPOINT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_CARTESIAN_SETPOINT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
  geometry_msgs::msg::Pose pose_msg_;

  // Must match the "cartesian_pose" sensor component name declared in kuka.ros2_control.xacro
  // (state interfaces x, y, z, a, b, c) when read_cartesian_pose is enabled.
  static constexpr char kSensorName[] = "cartesian_pose";
};
}  // namespace kuka_controllers
#endif  // KUKA_CARTESIAN_SETPOINT_BROADCASTER__KUKA_CARTESIAN_POSE_BROADCASTER_HPP_
