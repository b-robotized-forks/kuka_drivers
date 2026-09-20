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

#include "kuka_cartesian_setpoint_broadcaster/kuka_robot_status_broadcaster.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn RobotStatusBroadcaster::on_init()
{
  program_state_publisher_ = get_node()->create_publisher<std_msgs::msg::UInt8>(
    "~/program_state", rclcpp::SystemDefaultsQoS());
  speed_scaling_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/speed_scaling", rclcpp::SystemDefaultsQoS());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RobotStatusBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
RobotStatusBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const char * suffix : {"program_state", "speed_scaling_factor"})
  {
    config.names.emplace_back(std::string(kSensorName) + "/" + suffix);
  }
  return config;
}

controller_interface::CallbackReturn RobotStatusBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotStatusBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotStatusBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotStatusBroadcaster::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  program_state_msg_.data =
    static_cast<uint8_t>(state_interfaces_[0].get_optional().value_or(0.0));
  speed_scaling_msg_.data = state_interfaces_[1].get_optional().value_or(0.0);

  program_state_publisher_->publish(program_state_msg_);
  speed_scaling_publisher_->publish(speed_scaling_msg_);

  return controller_interface::return_type::OK;
}
}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::RobotStatusBroadcaster, controller_interface::ControllerInterface)
