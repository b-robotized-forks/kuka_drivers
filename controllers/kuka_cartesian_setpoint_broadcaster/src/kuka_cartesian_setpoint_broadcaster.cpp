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

#include "tf2/LinearMath/Quaternion.h"

#include "kuka_cartesian_setpoint_broadcaster/kuka_cartesian_setpoint_broadcaster.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn CartesianSetpointBroadcaster::on_init()
{
  pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::Pose>(
    "~/cartesian_setpoint", rclcpp::SystemDefaultsQoS());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianSetpointBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
CartesianSetpointBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const char * suffix : {"x", "y", "z", "a", "b", "c"})
  {
    config.names.emplace_back(std::string(kSensorName) + "/" + suffix);
  }
  return config;
}

controller_interface::CallbackReturn CartesianSetpointBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianSetpointBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianSetpointBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianSetpointBroadcaster::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // x, y, z arrive from the hardware interface in millimetres (KUKA RSI convention);
  // geometry_msgs/Pose expects metres.
  const double x_mm = state_interfaces_[0].get_optional().value_or(0.0);
  const double y_mm = state_interfaces_[1].get_optional().value_or(0.0);
  const double z_mm = state_interfaces_[2].get_optional().value_or(0.0);
  // a, b, c (KUKA ABC Euler angles) arrive already converted to radians.
  const double a_rad = state_interfaces_[3].get_optional().value_or(0.0);
  const double b_rad = state_interfaces_[4].get_optional().value_or(0.0);
  const double c_rad = state_interfaces_[5].get_optional().value_or(0.0);

  pose_msg_.position.x = x_mm / 1000.0;
  pose_msg_.position.y = y_mm / 1000.0;
  pose_msg_.position.z = z_mm / 1000.0;

  // KUKA's ABC convention is an intrinsic Z-Y'-X'' Euler rotation (A about Z, then B about the
  // new Y, then C about the newest X), which is exactly tf2::Quaternion::setRPY's
  // yaw-pitch-roll composition with yaw=A, pitch=B, roll=C.
  tf2::Quaternion orientation;
  orientation.setRPY(c_rad, b_rad, a_rad);
  pose_msg_.orientation.x = orientation.x();
  pose_msg_.orientation.y = orientation.y();
  pose_msg_.orientation.z = orientation.z();
  pose_msg_.orientation.w = orientation.w();

  pose_publisher_->publish(pose_msg_);

  return controller_interface::return_type::OK;
}
}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::CartesianSetpointBroadcaster, controller_interface::ControllerInterface)
