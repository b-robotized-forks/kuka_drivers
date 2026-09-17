// Copyright 2023 KUKA Hungaria Kft.
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

#include <algorithm>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_rsi_driver/hardware_interface_rsi_base.hpp"

namespace kuka_rsi_driver
{
CallbackReturn KukaRSIHardwareInterfaceBase::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  const std::string current_interface_name(kCurrentInterfaceName);
  has_current_interface_ =
    !info_.joints.empty() &&
    std::any_of(
      info_.joints[0].state_interfaces.cbegin(), info_.joints[0].state_interfaces.cend(),
      [&current_interface_name](const auto & state_interface)
      { return state_interface.name == current_interface_name; });
  hw_current_states_.resize(has_current_interface_ ? info_.joints.size() : 0, 0.0);

  for (const auto & joint : info_.joints)
  {
    bool interfaces_ok = CheckJointInterfaces(joint, has_current_interface_);
    if (!interfaces_ok)
    {
      return CallbackReturn::ERROR;
    }
  }

  // Check gpio components size
  if (info_.gpios.size() != 1)
  {
    RCLCPP_FATAL(logger_, "expecting exactly 1 gpio component");
    return CallbackReturn::ERROR;
  }
  const auto & gpio = info_.gpios[0];
  // Check gpio component name
  if (gpio.name != hardware_interface::IO_PREFIX)
  {
    RCLCPP_FATAL(logger_, "expecting gpio component called \"gpio\" first");
    return CallbackReturn::ERROR;
  }

  // Save the mapping of GPIO states to commands
  for (const auto & command_interface : gpio.command_interfaces)
  {
    // Find the corresponding state interface for each command interface and connect them based on
    // their names
    auto it = std::find_if(
      gpio.state_interfaces.begin(), gpio.state_interfaces.end(),
      [&command_interface](const hardware_interface::InterfaceInfo & state_interface)
      { return state_interface.name == command_interface.name; });
    if (it != gpio.state_interfaces.end())
    {
      gpio_states_to_commands_map_.push_back(std::distance(gpio.state_interfaces.begin(), it));
    }
    else
    {
      gpio_states_to_commands_map_.push_back(-1);  // Not found, use -1 as a placeholder
    }
  }

  hw_gpio_states_.resize(gpio.state_interfaces.size(), 0.0);
  hw_gpio_commands_.resize(gpio.command_interfaces.size(), 0.0);

  is_active_ = false;
  msg_received_ = false;

  // For plain RSI setup, there is no event broadcaster from the server, server events are published
  // based on HWIF logic to enable reactivation after an error
  // Locking is taken care of in resource manager (read, write, on_activate, on_deactivate)
  server_state_ = static_cast<double>(kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KukaRSIHardwareInterfaceBase::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
  }

  if (has_current_interface_)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(
        info_.joints[i].name, std::string(kCurrentInterfaceName), &hw_current_states_[i]);
    }
  }

  for (size_t i = 0; i < info_.gpios[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, info_.gpios[0].state_interfaces[i].name, &hw_gpio_states_[i]);
  }

  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaRSIHardwareInterfaceBase::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }

  for (size_t i = 0; i < info_.gpios[0].command_interfaces.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, info_.gpios[0].command_interfaces[i].name,
      &hw_gpio_commands_[i]);
  }

  return command_interfaces;
}

CallbackReturn KukaRSIHardwareInterfaceBase::on_cleanup(const rclcpp_lifecycle::State &)
{
  robot_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

return_type KukaRSIHardwareInterfaceBase::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // The first packet is received at activation, Read() should not be called before
  // Add short sleep to avoid RT thread eating CPU
  if (!is_active_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return return_type::OK;
  }

  Read(READ_TIMEOUT_MS);
  return return_type::OK;
}

return_type KukaRSIHardwareInterfaceBase::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // If control is not started or a request is missed, do not send back anything
  if (!msg_received_)
  {
    return return_type::OK;
  }

  Write();

  return return_type::OK;
}

bool KukaRSIHardwareInterfaceBase::SetupRobot(
  kuka::external::control::kss::Configuration & config,
  std::unique_ptr<kuka::external::control::EventHandler> event_handler,
  std::unique_ptr<kuka::external::control::kss::IEventHandlerExtension> extension)
{
  RCLCPP_INFO(logger_, "Setting up robot...");

  ConfigureJoints(config);

  RCLCPP_INFO(
    logger_, info_.gpios[0].command_interfaces.empty() ? "No GPIO command interfaces configured"
                                                       : "Configured GPIO commands:");
  for (const auto & gpio_command : info_.gpios[0].command_interfaces)
  {
    RCLCPP_INFO(
      logger_, "Name: %s, Data type: %s, Initial value: %s, Enable limits: %s, Min: %s, Max: %s",
      gpio_command.name.c_str(), gpio_command.data_type.c_str(), gpio_command.initial_value.c_str(),
      gpio_command.enable_limits ? "true" : "false", gpio_command.min.c_str(),
      gpio_command.max.c_str());

    // TODO(Komaromi): Add size and parameters
    config.gpio_command_configs.emplace_back(ParseGPIOConfig(gpio_command));
  }

  RCLCPP_INFO(
    logger_, info_.gpios[0].state_interfaces.empty() ? "No GPIO state interfaces configured"
                                                     : "Configured GPIO states:");
  for (const auto & gpio_state : info_.gpios[0].state_interfaces)
  {
    RCLCPP_INFO(
      logger_, "Name: %s, Data type: %s, Initial value: %s, Enable limits: %s, Min: %s, Max: %s",
      gpio_state.name.c_str(), gpio_state.data_type.c_str(), gpio_state.initial_value.c_str(),
      gpio_state.enable_limits ? "true" : "false", gpio_state.min.c_str(), gpio_state.max.c_str());

    // TODO(Komaromi): Add size, and parameters
    config.gpio_state_configs.emplace_back(ParseGPIOConfig(gpio_state));
  }

  ConfigureMotionStateXml(config);

  CreateRobotInstance(config);

  if (event_handler != nullptr)
  {
    auto status = robot_ptr_->RegisterEventHandler(std::move(event_handler));
    if (status.return_code == kuka::external::control::ReturnCode::ERROR)
    {
      RCLCPP_ERROR(logger_, "Creating event observer failed: %s", status.message);
    }
  }

  if (extension != nullptr)
  {
    auto status = robot_ptr_->RegisterEventHandlerExtension(std::move(extension));
    if (status.return_code == kuka::external::control::ReturnCode::ERROR)
    {
      RCLCPP_INFO(logger_, "Creating event handler extension failed: %s", status.message);
    }
  }
  const auto setup = robot_ptr_->Setup();
  if (setup.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Setup failed: %s", setup.message);
    return false;
  }

  RCLCPP_INFO(logger_, "Robot setup successful!");

  return true;
}

void KukaRSIHardwareInterfaceBase::Read(const int64_t request_timeout)
{
  auto motion_state_status =
    robot_ptr_->ReceiveMotionState(std::chrono::milliseconds(request_timeout));
  msg_received_ = motion_state_status.return_code == kuka::external::control::ReturnCode::OK;
  if (msg_received_)
  {
    // record timestamp immediately after the motion state is received
    auto now = std::chrono::steady_clock::now();

    // measure interval since previous packet if available
    if (last_msg_received_time_ != std::chrono::steady_clock::time_point{})
    {
      auto interval = now - last_msg_received_time_;
      auto interval_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(interval);
      // determine expected cycle time enum
      double dt_ms = (prev_cycle_time_ == RsiCycleTime::RSI_12MS) ? 12.0 : 4.0;  // default to 4ms
      double low_thresh = dt_ms - 0.5;
      double high_thresh = dt_ms + 0.5;
      if (interval_ms.count() < low_thresh || interval_ms.count() > high_thresh)
      {
        RCLCPP_WARN(
          logger_,
          "Unexpected RSI state interval: %.3f ms (expected %.3f±0.5 ms), change in interpolation "
          "count %lu",
          interval_ms.count(), dt_ms, robot_ptr_->getIpoc() - last_ipoc_);
      }
    }
    // update stored time for both interval and control-latency calculations
    last_msg_received_time_ = now;

    const auto & req_message = robot_ptr_->GetLastMotionState();
    const auto & positions = req_message.GetMeasuredPositions();
    const auto & gpio_values = req_message.GetGPIOValues();

    std::copy(positions.cbegin(), positions.cend(), hw_states_.begin());
    if (has_current_interface_)
    {
      const auto & currents = req_message.GetMeasuredCurrents();
      std::copy(currents.cbegin(), currents.cend(), hw_current_states_.begin());
    }
    // Save IO states
    for (size_t i = 0; i < hw_gpio_states_.size(); i++)
    {
      auto value = gpio_values.at(i)->GetValue();
      if (value.has_value())
      {
        hw_gpio_states_[i] = value.value();
      }
      else
      {
        RCLCPP_ERROR(
          logger_, "GPIO value not set. No value type found for GPIO %s (Should be dead code)",
          gpio_values.at(i)->GetGPIOConfig()->GetName().c_str());
      }
    }

    if (robot_ptr_->getDelay() != 0)
    {
      packet_loss_count_++;
      RCLCPP_WARN(
        logger_,
        "Packet loss registered, number of lost packets: %lu, continuous packet losses: %lu",
        packet_loss_count_, robot_ptr_->getDelay());
    }

    last_ipoc_ = robot_ptr_->getIpoc();
  }
  else
  {
    RCLCPP_ERROR(logger_, "Failed to receive motion state %s", motion_state_status.message);
    set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
  }

  std::lock_guard<std::mutex> lk(event_mutex_);
  server_state_ = static_cast<double>(last_event_);
}

void KukaRSIHardwareInterfaceBase::set_server_event(kuka_drivers_core::HardwareEvent event)
{
  std::lock_guard<std::mutex> lk(event_mutex_);
  last_event_ = event;
}

bool KukaRSIHardwareInterfaceBase::CheckJointInterfaces(
  const hardware_interface::ComponentInfo & joint, bool expect_current) const
{
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(logger_, "Expecting exactly 1 command interface");
    return false;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(logger_, "Expecting only POSITION command interface");
    return false;
  }

  const std::size_t expected_state_interfaces = expect_current ? 2 : 1;
  if (joint.state_interfaces.size() != expected_state_interfaces)
  {
    RCLCPP_FATAL(
      logger_, "Expecting exactly %zu state interface(s)", expected_state_interfaces);
    return false;
  }

  const bool has_position = std::any_of(
    joint.state_interfaces.cbegin(), joint.state_interfaces.cend(),
    [](const auto & state_interface)
    { return state_interface.name == hardware_interface::HW_IF_POSITION; });
  if (!has_position)
  {
    RCLCPP_FATAL(logger_, "Expecting a POSITION state interface");
    return false;
  }

  if (expect_current)
  {
    const std::string current_interface_name(kCurrentInterfaceName);
    const bool has_current = std::any_of(
      joint.state_interfaces.cbegin(), joint.state_interfaces.cend(),
      [&current_interface_name](const auto & state_interface)
      { return state_interface.name == current_interface_name; });
    if (!has_current)
    {
      RCLCPP_FATAL(
        logger_,
        "Every joint must declare a CURRENT state interface if any joint declares one");
      return false;
    }
  }

  return true;
}

void KukaRSIHardwareInterfaceBase::CopyGPIOStatesToCommands()
{
  for (size_t i = 0; i < gpio_states_to_commands_map_.size(); i++)
  {
    if (gpio_states_to_commands_map_[i] != -1)
    {
      hw_gpio_commands_[i] = hw_gpio_states_[gpio_states_to_commands_map_[i]];
    }
  }
}

kuka::external::control::kss::GPIOConfiguration KukaRSIHardwareInterfaceBase::ParseGPIOConfig(
  const hardware_interface::InterfaceInfo & info)
{
  kuka::external::control::kss::GPIOConfiguration gpio_config;
  gpio_config.name = info.name;
  gpio_config.enable_limits = info.enable_limits;
  // TODO(komaromi): This might not work from Kilted kaiju onward the get_optional function in the
  // handle since it is only accepting double and bool
  if (info.data_type == "BOOL" || info.data_type == "bool")
  {
    gpio_config.value_type = kuka::external::control::GPIOValueType::BOOL;
  }
  else if (info.data_type == "DOUBLE" || info.data_type == "double")
  {
    gpio_config.value_type = kuka::external::control::GPIOValueType::DOUBLE;
  }
  else if (info.data_type == "LONG")
  {
    gpio_config.value_type = kuka::external::control::GPIOValueType::LONG;
  }
  else
  {
    gpio_config.value_type = kuka::external::control::GPIOValueType::UNSPECIFIED;
  }

  if (!info.initial_value.empty())
  {
    try
    {
      gpio_config.initial_value = std::stod(info.initial_value);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(
        logger_, "Initial_value is not valid number, it is set to 0. Exception: %s", ex.what());
      gpio_config.initial_value = 0.0;  // If initial_value is not a valid number, set to 0.0
    }
  }
  else
  {
    // TODO(Komaromi): Should this be set to 0?
    gpio_config.initial_value = 0.0;  // If initial_value is empty, set to 0.0
  }
  if (!info.min.empty())
  {
    try
    {
      gpio_config.min_value = std::stod(info.min);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(
        logger_, "Min_value is not valid number, limits not used. Exception: %s", ex.what());
      gpio_config.enable_limits = false;  // If min_value is not a valid number, disable limits
    }
  }
  else
  {
    gpio_config.enable_limits = false;  // If min_value is empty, disable limits
  }
  if (!info.max.empty())
  {
    try
    {
      gpio_config.max_value = std::stod(info.max);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(
        logger_, "Max_value is not valid number, limits not used. Exception: %s", ex.what());
      gpio_config.enable_limits = false;  // If max_value is not a valid number, disable limits
    }
  }
  else
  {
    gpio_config.enable_limits = false;  // If max_value is empty, disable limits
  }
  if (gpio_config.enable_limits && (gpio_config.min_value > gpio_config.max_value))
  {
    RCLCPP_WARN(
      logger_, "Min value is greater than max value, limits not used. Min: %f, Max: %f",
      gpio_config.min_value, gpio_config.max_value);
    gpio_config.enable_limits = false;  // If min_value is greater than or equal to max_value,
                                        // disable limits
  }
  return gpio_config;
}

CallbackReturn KukaRSIHardwareInterfaceBase::extended_activation(const rclcpp_lifecycle::State &)
{
  ResetDiagnostics();

  if (status_manager_.IsEmergencyStopActive())
  {
    RCLCPP_ERROR(logger_, "Emergency stop is active. Cannot activate hardware interface.");
    return CallbackReturn::FAILURE;
  }

  if (!status_manager_.IsKrcInExtMode())
  {
    RCLCPP_ERROR(logger_, "KRC not in EXT. Switch to EXT to activate.");
    return CallbackReturn::FAILURE;
  }

  if (!status_manager_.DrivesPowered())
  {
    RCLCPP_INFO(logger_, "Turning on drives");
    robot_ptr_->TurnOnDrives();

    // Wait for drives to be powered up
    auto start_time = std::chrono::steady_clock::now();
    while (!status_manager_.DrivesPowered())
    {
      if (
        std::chrono::steady_clock::now() - start_time >
        KukaRSIHardwareInterfaceBase::DRIVES_POWERED_TIMEOUT)
      {
        RCLCPP_ERROR(logger_, "Timeout waiting for drives to power on. Check robot state.");
        return CallbackReturn::FAILURE;
      }
      status_manager_.UpdateStateInterfaces();
      std::this_thread::sleep_for(KukaRSIHardwareInterfaceBase::DRIVES_POWERED_CHECK_INTERVAL);
    }
    RCLCPP_INFO(logger_, "Drives successfully powered on");
  }

  // Set control mode and cycle time before sending Start request
  ChangeCycleTime();

  const auto control_mode =
    static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_);

  kuka::external::control::Status control_status = robot_ptr_->StartControlling(control_mode);
  if (control_status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(logger_, "Starting external control failed: %s", control_status.message);
    return CallbackReturn::FAILURE;
  }

  prev_control_mode_ = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);

  // We must first receive the initial position of the robot
  // We set a longer timeout, since the first message might not arrive all that fast
  Read(5 * READ_TIMEOUT_MS);
  std::copy(hw_states_.cbegin(), hw_states_.cend(), hw_commands_.begin());
  CopyGPIOStatesToCommands();
  Write();

  msg_received_ = false;
  is_active_ = true;

  RCLCPP_INFO(logger_, "Received position data from robot controller!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterfaceBase::extended_deactivation(const rclcpp_lifecycle::State &)
{
  if (msg_received_)
  {
    RCLCPP_INFO(logger_, "Deactivating hardware interface by sending stop signal");

    // StopControlling sometimes calls a blocking read, which could conflict with the read() method,
    // but resource manager handles locking (resources_lock_), so is not necessary here
    robot_ptr_->StopControlling();
  }
  else
  {
    RCLCPP_INFO(logger_, "Message not received, but stop requested. Cancelling RSI program.");
    robot_ptr_->CancelRsiProgram();
  }
  is_active_ = false;
  msg_received_ = false;

  if (status_manager_.DrivesPowered())
  {
    RCLCPP_INFO(logger_, "Turning off drives");
    robot_ptr_->TurnOffDrives();

    // Wait for drives to be powered off
    auto start_time = std::chrono::steady_clock::now();
    while (status_manager_.DrivesPowered())
    {
      if (
        std::chrono::steady_clock::now() - start_time >
        KukaRSIHardwareInterfaceBase::DRIVES_POWERED_TIMEOUT)
      {
        RCLCPP_ERROR(logger_, "Timeout waiting for drives to power off. Check robot state.");
        // Return success, as drives off signal is not received in Office mode for iiQKA.OS2
        status_manager_.UpdateStateInterfaces();
        return CallbackReturn::SUCCESS;
      }
      status_manager_.UpdateStateInterfaces();
      std::this_thread::sleep_for(KukaRSIHardwareInterfaceBase::DRIVES_POWERED_CHECK_INTERVAL);
    }
    RCLCPP_INFO(logger_, "Drives successfully powered off");
  }
  return CallbackReturn::SUCCESS;
}

void KukaRSIHardwareInterfaceBase::Write()
{
  // Write values to hardware interface
  auto & control_signal = robot_ptr_->GetControlSignal();
  control_signal.AddJointPositionValues(hw_commands_.cbegin(), hw_commands_.cend());
  control_signal.AddGPIOValues(hw_gpio_commands_.cbegin(), hw_gpio_commands_.cend());

  // measure elapsed time since last motion state message
  // No need to check msg_received_ here, as Write() is only called when msg_received_ is true
  auto now = std::chrono::steady_clock::now();
  auto elapsed =
    std::chrono::duration_cast<std::chrono::microseconds>(now - last_msg_received_time_);
  // if the delay exceeds threshold, flag as warning
  if (elapsed > KukaRSIHardwareInterfaceBase::kWarningThreshold)
  {
    RCLCPP_WARN(
      logger_,
      "Slow response: %ld us elapsed between motion state was received and control signal sent",
      static_cast<uint64_t>(elapsed.count()));
  }

  auto send_reply_status = robot_ptr_->SendControlSignal();

  if (send_reply_status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Sending reply failed: %s", send_reply_status.message);
    throw std::runtime_error("Error sending reply");
  }
}

void KukaRSIHardwareInterfaceBase::ResetDiagnostics()
{
  // Reset diagnostics related variables
  packet_loss_count_ = 0;
  last_ipoc_ = 0;
  last_msg_received_time_ = std::chrono::steady_clock::time_point{};
}

kuka::external::control::Status KukaRSIHardwareInterfaceBase::ChangeCycleTime()
{
  const RsiCycleTime cycle_time = static_cast<RsiCycleTime>(cycle_time_command_);

  if (prev_cycle_time_ != cycle_time)
  {
    RCLCPP_INFO(
      logger_, "Changing RSI cycle time to %s",
      kuka::external::control::kss::CycleTimeToString(cycle_time));
    auto status = robot_ptr_->SetCycleTime(cycle_time);
    if (status.return_code != kuka::external::control::ReturnCode::OK)
    {
      return status;
    }
    prev_cycle_time_ = cycle_time;
  }

  return kuka::external::control::Status(kuka::external::control::ReturnCode::OK);
}

void KukaRSIHardwareInterfaceBase::initialize_command_interfaces(
  kuka_drivers_core::ControlMode control_mode, RsiCycleTime cycle_time)
{
  prev_control_mode_ = control_mode;
  prev_cycle_time_ = cycle_time;
  hw_control_mode_command_ = static_cast<double>(control_mode);
  cycle_time_command_ = static_cast<double>(cycle_time);
}

void KukaRSIHardwareInterfaceBase::ConfigureJoints(
  kuka::external::control::kss::Configuration & config) const
{
  using JC = kuka::external::control::kss::JointConfiguration;

  config.dof = info_.joints.size();
  config.joint_configs.reserve(info_.joints.size());

  for (const auto & joint : info_.joints)
  {
    // Default to revolute joints
    const auto type_it = joint.parameters.find(std::string(kTypeParamValue));
    const auto type =
      (type_it == joint.parameters.end()) ? JC::Type::REVOLUTE : JC::ToType(type_it->second);

    // Default to internal joints
    const auto external_it = joint.parameters.find(std::string(kIsExternalParamValue));
    const bool is_external =
      (external_it == joint.parameters.end()) ? false : external_it->second == "true";

    config.joint_configs.emplace_back(joint.name, type, is_external);

    RCLCPP_INFO_STREAM(
      logger_, "Configured joint \"" << joint.name << "\": type=" << JC::TypeToString(type)
                                     << ", external=" << (is_external ? "true" : "false"));
  }
}

void KukaRSIHardwareInterfaceBase::ConfigureMotionStateXml(
  kuka::external::control::kss::Configuration & config) const
{
  // Leave the SDK's default motion-state XML layout (position + GPIO only) untouched unless the
  // URDF opted into a "current" state interface for every joint (checked in on_init() via
  // CheckJointInterfaces()) -- this requires the robot's RSI config to also transmit the
  // internal MACur/MECur elements (motor current for A1-A6 / E1-E6), see the aip_cell_
  // configuration docs for how to enable that on the KRC side.
  if (!has_current_interface_)
  {
    return;
  }

  using MSF = kuka::external::control::kss::MotionStateJointFieldConfiguration;
  using MSST = kuka::external::control::kss::MotionStateSignalType;
  using MSXFT = kuka::external::control::kss::MotionStateXmlFieldType;

  kuka::external::control::kss::MotionStateXmlConfiguration xml_config;

  xml_config.gpio_xml_attributes.reserve(config.gpio_state_configs.size());
  for (const auto & gpio : config.gpio_state_configs)
  {
    xml_config.gpio_xml_attributes.push_back(gpio.name);
  }

  // Unlike RIst/AIPos/EIPos/Delay, MACur/MECur are not part of the RSI wire format's fixed
  // internal-element block -- they appear wherever they were declared in the robot-side SEND
  // element list, which on this cell's config is AFTER the GPIO elements and right before Delay
  // (<RIst><AIPos><EIPos><GPIO><MACur><Delay><IPOC>). We therefore build joint position fields
  // and current fields as separate groups below and place them explicitly in field_order to
  // match -- if the robot's RSI config is ever redeclared with MACur elsewhere, this ordering
  // (and the explicit field_order built below) must be updated to match.
  std::size_t internal_idx = 1;
  std::size_t external_idx = 1;
  std::vector<MSF> internal_position_fields;
  std::vector<MSF> external_position_fields;
  std::vector<MSF> internal_current_fields;
  std::vector<MSF> external_current_fields;

  for (const auto & joint : config.joint_configs)
  {
    MSF position_field;
    position_field.joint_identifier = joint.name;
    position_field.signal_type = MSST::POSITION;

    MSF current_field;
    current_field.joint_identifier = joint.name;
    current_field.signal_type = MSST::CURRENT;

    if (joint.is_external)
    {
      position_field.xml_element = "EIPos";
      current_field.xml_element = "MECur";
      external_position_fields.push_back(std::move(position_field));
      external_current_fields.push_back(std::move(current_field));
    }
    else
    {
      position_field.xml_element = "AIPos";
      current_field.xml_element = "MACur";
      internal_position_fields.push_back(std::move(position_field));
      internal_current_fields.push_back(std::move(current_field));
    }
  }

  for (auto & field : internal_position_fields)
  {
    field.xml_attribute = "A" + std::to_string(internal_idx++);
    xml_config.joint_fields.push_back(std::move(field));
  }
  for (auto & field : external_position_fields)
  {
    field.xml_attribute = "E" + std::to_string(external_idx++);
    xml_config.joint_fields.push_back(std::move(field));
  }
  const std::size_t num_position_fields = xml_config.joint_fields.size();

  internal_idx = 1;
  external_idx = 1;
  for (auto & field : internal_current_fields)
  {
    field.xml_attribute = "A" + std::to_string(internal_idx++);
    xml_config.joint_fields.push_back(std::move(field));
  }
  for (auto & field : external_current_fields)
  {
    field.xml_attribute = "E" + std::to_string(external_idx++);
    xml_config.joint_fields.push_back(std::move(field));
  }

  // Explicit parse order matching this cell's robot-side SEND declaration order:
  // Cartesian, position joints, GPIO, current joints, Delay (IPOC is always appended last by
  // the SDK). See the comment above for why current fields can't just be grouped with position
  // fields the way the SDK's own default ordering would do.
  xml_config.field_order.reserve(2 + xml_config.joint_fields.size() + xml_config.gpio_xml_attributes.size());
  xml_config.field_order.push_back({MSXFT::CARTESIAN, 0});
  for (std::size_t i = 0; i < num_position_fields; ++i)
  {
    xml_config.field_order.push_back({MSXFT::JOINT, i});
  }
  for (std::size_t i = 0; i < xml_config.gpio_xml_attributes.size(); ++i)
  {
    xml_config.field_order.push_back({MSXFT::GPIO, i});
  }
  for (std::size_t i = num_position_fields; i < xml_config.joint_fields.size(); ++i)
  {
    xml_config.field_order.push_back({MSXFT::JOINT, i});
  }
  xml_config.field_order.push_back({MSXFT::DELAY, 0});

  config.motion_state_xml_config = std::move(xml_config);
}

}  // namespace kuka_rsi_driver
