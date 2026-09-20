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

#ifndef KUKA_CARTESIAN_SETPOINT_BROADCASTER__KUKA_ROBOT_STATUS_BROADCASTER_HPP_
#define KUKA_CARTESIAN_SETPOINT_BROADCASTER__KUKA_ROBOT_STATUS_BROADCASTER_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "kuka_cartesian_setpoint_broadcaster/visibility_control.h"

namespace kuka_controllers
{
// Broadcasts the KRC's program state ($PRO_STATE, via the RSI "Status" object) and program
// override ($OV_PRO, via the RSI "OV_PRO" object) -- see the "robot_status" sensor component in
// kuka.ros2_control.xacro.
class RobotStatusBroadcaster : public controller_interface::ControllerInterface
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
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr program_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_scaling_publisher_;
  std_msgs::msg::UInt8 program_state_msg_;
  std_msgs::msg::Float64 speed_scaling_msg_;

  // Must match the "robot_status" sensor component name declared in kuka.ros2_control.xacro
  // (state interfaces program_state, speed_scaling_factor) when read_robot_status is enabled.
  static constexpr char kSensorName[] = "robot_status";
};
}  // namespace kuka_controllers
#endif  // KUKA_CARTESIAN_SETPOINT_BROADCASTER__KUKA_ROBOT_STATUS_BROADCASTER_HPP_
