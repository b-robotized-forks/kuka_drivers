#ifndef KUKA_ROBOT_STATUS_BROADCASTER__KUKA_ROBOT_STATUS_BROADCASTER_HPP_
#define KUKA_ROBOT_STATUS_BROADCASTER__KUKA_ROBOT_STATUS_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "kuka_robot_status_broadcaster/visibility_control.h"

namespace kuka_controllers
{
// Broadcasts the KRC's program state ($PRO_STATE, via a custom RSIVisual "Status" object) and
// program override/speed scaling ($OV_PRO, via "OV_PRO") as ROS topics -- see the "robot_status"
// sensor component in the robot's URDF and kuka_rsi_driver's kRobotStatusSensorName.
class RobotStatusBroadcaster : public controller_interface::ControllerInterface
{
public:
  KUKA_ROBOT_STATUS_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  KUKA_ROBOT_STATUS_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  KUKA_ROBOT_STATUS_BROADCASTER_PUBLIC controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  KUKA_ROBOT_STATUS_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_ROBOT_STATUS_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_ROBOT_STATUS_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_ROBOT_STATUS_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

private:
  // Must match the "robot_status" sensor component name declared in the URDF (state interfaces
  // program_state, speed_scaling_factor) -- see kuka_rsi_driver's kRobotStatusSensorName.
  static constexpr char kSensorName[] = "robot_status";

  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::UInt8>>
    program_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>
    speed_scaling_publisher_;
};
}  // namespace kuka_controllers
#endif  // KUKA_ROBOT_STATUS_BROADCASTER__KUKA_ROBOT_STATUS_BROADCASTER_HPP_
