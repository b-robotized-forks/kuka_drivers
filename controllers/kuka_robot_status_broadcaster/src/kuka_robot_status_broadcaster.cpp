#include "kuka_robot_status_broadcaster/kuka_robot_status_broadcaster.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn RobotStatusBroadcaster::on_init()
{
  auto program_state_publisher = get_node()->create_publisher<std_msgs::msg::UInt8>(
    "~/program_state", rclcpp::SystemDefaultsQoS());
  program_state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::UInt8>>(
      program_state_publisher);

  auto speed_scaling_publisher = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/speed_scaling", rclcpp::SystemDefaultsQoS());
  speed_scaling_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
      speed_scaling_publisher);

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
  const auto program_state =
    static_cast<uint8_t>(state_interfaces_[0].get_optional().value_or(0.0));
  const auto speed_scaling = state_interfaces_[1].get_optional().value_or(0.0);

  if (program_state_publisher_->trylock())
  {
    program_state_publisher_->msg_.data = program_state;
    program_state_publisher_->unlockAndPublish();
  }

  if (speed_scaling_publisher_->trylock())
  {
    speed_scaling_publisher_->msg_.data = speed_scaling;
    speed_scaling_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}
}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::RobotStatusBroadcaster, controller_interface::ControllerInterface)
