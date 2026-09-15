// Copyright 2020 Zoltán Rési
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

#include <memory>

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "kuka_sunrise_fri_driver/hardware_interface.hpp"

namespace kuka_sunrise_fri_driver
{
CallbackReturn KukaFRIHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  fri_connection_ =
    std::make_shared<FRIConnection>([this] { this->onError(); }, [this] { this->onError(); });

  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  controller_ip_ = info_.hardware_parameters.at("controller_ip");
  client_ip_ = info_.hardware_parameters.at("client_ip");
  client_port_ = std::stoi(info_.hardware_parameters.at("client_port"));

  hw_position_states_.resize(info_.joints.size());
  hw_commanded_position_states_.resize(info_.joints.size());
  hw_position_commands_.resize(info_.joints.size());
  hw_stiffness_commands_.resize(info_.joints.size());
  hw_damping_commands_.resize(info_.joints.size());
  hw_torque_states_.resize(info_.joints.size());
  hw_ext_torque_states_.resize(info_.joints.size());
  hw_torque_commands_.resize(info_.joints.size());

  if (info_.gpios.size() != 1)
  {
    RCLCPP_FATAL(rclcpp::get_logger("KukaFRIHardwareInterface"), "expecting exactly 1 GPIO");
    return CallbackReturn::ERROR;
  }

  if (info_.gpios[0].command_interfaces.size() > 10 || info_.gpios[0].state_interfaces.size() > 10)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("KukaFRIHardwareInterface"),
      "A maximum of 10 inputs and outputs can be registered to FRI");
    return CallbackReturn::ERROR;
  }

  for (const auto & state_if : info_.gpios[0].state_interfaces)
  {
    gpio_outputs_.emplace_back(state_if.name, getType(state_if.data_type), robotState());
  }
  for (const auto & output : gpio_outputs_)
  {
    gpio_output_names_.push_back(
      std::string(hardware_interface::IO_PREFIX) + "/" + output.getName());
  }

  for (const auto & command_if : info_.gpios[0].command_interfaces)
  {
    const IOTypes io_type = getType(command_if.data_type);
    // Boolean GPIO commands carry "true"/"false" in initial_value (required by
    // hardware_interface::Handle's own bool parsing), not a stod-able number.
    const double initial_value =
      io_type == IOTypes::BOOLEAN
        ? static_cast<double>(hardware_interface::parse_bool(command_if.initial_value))
        : std::stod(command_if.initial_value);
    gpio_inputs_.emplace_back(command_if.name, io_type, robotCommand(), initial_value);
  }
  for (const auto & input : gpio_inputs_)
  {
    gpio_input_names_.push_back(
      std::string(hardware_interface::IO_PREFIX) + "/" + input.getName());
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 4)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"), "expecting exactly 4 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'POSITION' command interface as first");
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_STIFFNESS)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'STIFFNESS' command interface as second");
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_DAMPING)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'DAMPING' command interface as third");
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[3].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'EFFORT' command interface as fourth");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 4)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"), "expecting exactly 4 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'POSITION' state interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'EFFORT' state interface as second");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EXTERNAL_TORQUE)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'EXTERNAL_TORQUE' state interface as third");
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[3].name != hardware_interface::HW_IF_COMMANDED_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'COMMANDED_POSITION' state interface as fourth");
      return CallbackReturn::ERROR;
    }
  }

  joint_position_state_names_.resize(info_.joints.size());
  joint_effort_state_names_.resize(info_.joints.size());
  joint_external_torque_state_names_.resize(info_.joints.size());
  joint_commanded_position_state_names_.resize(info_.joints.size());
  joint_position_command_names_.resize(info_.joints.size());
  joint_stiffness_command_names_.resize(info_.joints.size());
  joint_damping_command_names_.resize(info_.joints.size());
  joint_effort_command_names_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const std::string & name = info_.joints[i].name;
    joint_position_state_names_[i] = name + "/" + hardware_interface::HW_IF_POSITION;
    joint_effort_state_names_[i] = name + "/" + hardware_interface::HW_IF_EFFORT;
    joint_external_torque_state_names_[i] =
      name + "/" + hardware_interface::HW_IF_EXTERNAL_TORQUE;
    joint_commanded_position_state_names_[i] =
      name + "/" + hardware_interface::HW_IF_COMMANDED_POSITION;
    joint_position_command_names_[i] = name + "/" + hardware_interface::HW_IF_POSITION;
    joint_stiffness_command_names_[i] = name + "/" + hardware_interface::HW_IF_STIFFNESS;
    joint_damping_command_names_[i] = name + "/" + hardware_interface::HW_IF_DAMPING;
    joint_effort_command_names_[i] = name + "/" + hardware_interface::HW_IF_EFFORT;
  }

  fixed_state_interfaces_ = {
    {std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::SESSION_STATE,
     &robot_state_.session_state_},
    {std::string(hardware_interface::FRI_STATE_PREFIX) + "/" +
       hardware_interface::CONNECTION_QUALITY,
     &robot_state_.connection_quality_},
    {std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::SAFETY_STATE,
     &robot_state_.safety_state_},
    {std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::COMMAND_MODE,
     &robot_state_.command_mode_},
    {std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::CONTROL_MODE,
     &robot_state_.control_mode_},
    {std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::OPERATION_MODE,
     &robot_state_.operation_mode_},
    {std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::DRIVE_STATE,
     &robot_state_.drive_state_},
    {std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::OVERLAY_TYPE,
     &robot_state_.overlay_type_},
    {std::string(hardware_interface::FRI_STATE_PREFIX) + "/" +
       hardware_interface::TRACKING_PERFORMANCE,
     &robot_state_.tracking_performance_},
  };
  server_state_name_ =
    std::string(hardware_interface::STATE_PREFIX) + "/" + hardware_interface::SERVER_STATE;

  control_mode_command_name_ =
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::CONTROL_MODE;
  receive_multiplier_command_name_ =
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::RECEIVE_MULTIPLIER;
  send_period_command_name_ =
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::SEND_PERIOD;

  RCLCPP_INFO(
    rclcpp::get_logger("KukaFRIHardwareInterface"),
    "Init successful with controller ip: %s and client ip: %s:%i", controller_ip_.c_str(),
    client_ip_.c_str(), client_port_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  // Set up UDP connection (UDP replier on client)
  if (!client_application_.connect(client_port_, controller_ip_.c_str()))
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not set up UDP connection");
    return CallbackReturn::FAILURE;
  }

  // Set up TCP connection necessary for configuration
  if (!fri_connection_->connect(controller_ip_.c_str(), TCP_SERVER_PORT))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"),
      "Could not initialize TCP connection to controller");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("KukaFRIHardwareInterface"), "Successfully connected to FRI application");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  client_application_.disconnect();

  if (!fri_connection_->disconnect())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"),
      "Could not close TCP connection to controller");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // FRI config cannot be set during hardware interface configuration, as the controller cannot
  // modify the cmd interface until the hardware reached the configured state
  // write() is no longer called in inactive state, so we can only set config during activation

  control_mode_ = get_command<double>(control_mode_command_name_);
  receive_multiplier_ = get_command<double>(receive_multiplier_command_name_);
  send_period_ms_ = get_command<double>(send_period_command_name_);

  if (!fri_connection_->setFRIConfig(
        client_ip_, client_port_, static_cast<int>(send_period_ms_),
        static_cast<int>(receive_multiplier_)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not set FRI config");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("KukaFRIHardwareInterface"), "Successfully set FRI config");

  // Set control mode before starting motion - not even the impedance attributes can be changed in
  // active state
  switch (static_cast<kuka_drivers_core::ControlMode>(control_mode_))
  {
    case kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL:
      fri_connection_->setPositionControlMode();
      fri_connection_->setClientCommandMode(ClientCommandModeID::POSITION_COMMAND_MODE);
      break;
    case kuka_drivers_core::ControlMode::JOINT_IMPEDANCE_CONTROL:
      fri_connection_->setJointImpedanceControlMode(hw_stiffness_commands_, hw_damping_commands_);
      fri_connection_->setClientCommandMode(ClientCommandModeID::POSITION_COMMAND_MODE);
      break;
    case kuka_drivers_core::ControlMode::JOINT_TORQUE_CONTROL:
      fri_connection_->setJointImpedanceControlMode(
        std::vector<double>(DOF, 0.0), std::vector<double>(DOF, 0.0));
      fri_connection_->setClientCommandMode(ClientCommandModeID::TORQUE_COMMAND_MODE);
      break;

    default:
      RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Unsupported control mode");
      return CallbackReturn::ERROR;
  }

  // startFRI and activateControl must be done on a different thread, as they would block the read
  // function due to new mutex and cause a timeout
  thread_running_ = true;
  std::thread init_thread(
    [&]
    {
      // Start FRI (in monitoring mode)
      if (!fri_connection_->startFRI())
      {
        RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not start FRI");
        thread_running_ = false;
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("KukaFRIHardwareInterface"), "Started FRI");
      fri_started_ = true;

      // Switch to commanding mode
      if (!fri_connection_->activateControl())
      {
        RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not activate control");
        thread_running_ = false;
        return;
      }
      control_activated_ = true;
      RCLCPP_INFO(rclcpp::get_logger("KukaFRIHardwareInterface"), "Activated control");
      thread_running_ = false;
    });

  // Keep cyclic communication, while the other thread is working
  while (thread_running_)
  {
    client_application_.client_app_read();
    client_application_.client_app_update();
    client_application_.client_app_write();
  }

  init_thread.join();

  if (!fri_started_ || !control_activated_)
  {
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!fri_connection_->deactivateControl())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not deactivate control");
    return CallbackReturn::ERROR;
  }
  control_activated_ = false;

  if (!fri_connection_->endFRI())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not end FRI");
    return CallbackReturn::ERROR;
  }
  fri_started_ = false;

  return CallbackReturn::SUCCESS;
}

void KukaFRIHardwareInterface::waitForCommand()
{
  // In COMMANDING_WAIT state, the controller and client sync commanded positions
  // Therefore the control signal should not be modified in this callback
  // TODO(Svastits): check for torque/impedance mode, where state can change
  hw_position_commands_ = hw_position_states_;
  rclcpp::Time stamp = ros_clock_.now();
  updateCommand(stamp);
}

void KukaFRIHardwareInterface::command()
{
  rclcpp::Time stamp = ros_clock_.now();
  if (++receive_counter_ == receive_multiplier_)
  {
    updateCommand(stamp);
    receive_counter_ = 0;
  }
}

hardware_interface::return_type KukaFRIHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Make sure to only call client app calls if not called from the other thread
  // This is relevant only for backward compatibility, as new mutex disables
  //  on_activate() and read() at the same time
  if (thread_running_)
  {
    active_read_ = false;
    return hardware_interface::return_type::OK;
  }

  if ((active_read_ = client_application_.client_app_read() == true))
  {
    // get the position and efforts and share them with exposed state interfaces
    const double * position = robotState().getMeasuredJointPosition();
    hw_position_states_.assign(position, position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    const double * torque = robotState().getMeasuredTorque();
    hw_torque_states_.assign(torque, torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    const double * external_torque = robotState().getExternalTorque();
    hw_ext_torque_states_.assign(
      external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

    std::copy(
      hw_position_commands_.begin(), hw_position_commands_.end(),
      hw_commanded_position_states_.begin());

    robot_state_.tracking_performance_ = robotState().getTrackingPerformance();
    robot_state_.session_state_ = robotState().getSessionState();
    robot_state_.connection_quality_ = robotState().getConnectionQuality();
    robot_state_.command_mode_ = robotState().getClientCommandMode();
    robot_state_.safety_state_ = robotState().getSafetyState();
    robot_state_.control_mode_ = robotState().getControlMode();
    robot_state_.operation_mode_ = robotState().getOperationMode();
    robot_state_.drive_state_ = robotState().getDriveState();
    robot_state_.overlay_type_ = robotState().getOverlayType();

    for (auto & output : gpio_outputs_)
    {
      output.getValue();
    }

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      set_state(joint_position_state_names_[i], hw_position_states_[i]);
      set_state(joint_effort_state_names_[i], hw_torque_states_[i]);
      set_state(joint_external_torque_state_names_[i], hw_ext_torque_states_[i]);
      set_state(joint_commanded_position_state_names_[i], hw_commanded_position_states_[i]);
    }
    for (size_t i = 0; i < gpio_output_names_.size(); i++)
    {
      set_state(gpio_output_names_[i], gpio_outputs_[i].getData());
    }
    for (const auto & [name, value_ptr] : fixed_state_interfaces_)
    {
      set_state(name, *value_ptr);
    }
  }

  // Modify state interface only in read
  std::lock_guard<std::mutex> lk(event_mutex_);
  server_state_ = static_cast<double>(last_event_);
  set_state(server_state_name_, server_state_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KukaFRIHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Make sure to only call client app calls if read has been called before
  // write() is no longer called before HWIF is activated, so it is not necessary to check for
  // control started
  if (!active_read_)
  {
    return hardware_interface::return_type::OK;
  }

  control_mode_ = get_command<double>(control_mode_command_name_);
  receive_multiplier_ = get_command<double>(receive_multiplier_command_name_);
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_position_commands_[i] = get_command<double>(joint_position_command_names_[i]);
    hw_stiffness_commands_[i] = get_command<double>(joint_stiffness_command_names_[i]);
    hw_damping_commands_[i] = get_command<double>(joint_damping_command_names_[i]);
    hw_torque_commands_[i] = get_command<double>(joint_effort_command_names_[i]);
  }
  for (size_t i = 0; i < gpio_input_names_.size(); i++)
  {
    gpio_inputs_[i].getData() = get_command<double>(gpio_input_names_[i]);
  }

  // Call the appropriate callback for the actual state (e.g. updateCommand)
  //  in active state this updates the command to be sent based on the command interfaces
  client_application_.client_app_update();

  if (!client_application_.client_app_write())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not send command to controller");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void KukaFRIHardwareInterface::updateCommand(const rclcpp::Time &)
{
  switch (static_cast<kuka_drivers_core::ControlMode>(control_mode_))
  {
    case kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL:
      [[fallthrough]];
    case kuka_drivers_core::ControlMode::JOINT_IMPEDANCE_CONTROL:
    {
      const double * joint_positions_ = hw_position_commands_.data();
      robotCommand().setJointPosition(joint_positions_);
      break;
    }
    case kuka_drivers_core::ControlMode::JOINT_TORQUE_CONTROL:
    {
      const double * joint_torques_ = hw_torque_commands_.data();
      const double * joint_pos = robotState().getMeasuredJointPosition();
      std::array<double, DOF> joint_pos_corr;
      std::copy(joint_pos, joint_pos + DOF, joint_pos_corr.begin());
      activateFrictionCompensation(joint_pos_corr.data());
      robotCommand().setJointPosition(joint_pos_corr.data());
      robotCommand().setTorque(joint_torques_);
      break;
    }
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "Unsupported control mode, exiting updateCommand");
      return;
  }

  for (auto & input : gpio_inputs_)
  {
    input.setValue();
  }
}

namespace
{
hardware_interface::InterfaceDescription MakeUnlistedInterface(
  const std::string & prefix, const std::string & name)
{
  hardware_interface::InterfaceInfo info{};
  info.name = name;
  info.initial_value = "0";
  return hardware_interface::InterfaceDescription(prefix, info);
}
}  // namespace

std::vector<hardware_interface::InterfaceDescription>
KukaFRIHardwareInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> descriptions;

  for (const auto & [name, value_ptr] : fixed_state_interfaces_)
  {
    // fixed_state_interfaces_ stores the already-prefixed full name; split it back out, since
    // InterfaceDescription wants prefix and name separately.
    const auto slash_pos = name.find('/');
    descriptions.push_back(
      MakeUnlistedInterface(name.substr(0, slash_pos), name.substr(slash_pos + 1)));
  }
  descriptions.push_back(
    MakeUnlistedInterface(hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE));

  return descriptions;
}

std::vector<hardware_interface::InterfaceDescription>
KukaFRIHardwareInterface::export_unlisted_command_interface_descriptions()
{
  return {
    MakeUnlistedInterface(hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE),
    MakeUnlistedInterface(
      hardware_interface::CONFIG_PREFIX, hardware_interface::RECEIVE_MULTIPLIER),
    MakeUnlistedInterface(hardware_interface::CONFIG_PREFIX, hardware_interface::SEND_PERIOD)};
}

// Friction compensation is activated only if the commanded and measured joint positions differ
void KukaFRIHardwareInterface::activateFrictionCompensation(double * values) const
{
  for (int i = 0; i < DOF; i++)
  {
    if (values[i] != 0.0)
    {
      values[i] -= (values[i] / fabs(values[i]) * 0.1);
    }
    else
    {
      values[i] -= 0.1;
    }
  }
}

void KukaFRIHardwareInterface::onError()
{
  std::lock_guard<std::mutex> lk(event_mutex_);
  last_event_ = kuka_drivers_core::HardwareEvent::ERROR;
  RCLCPP_ERROR(
    rclcpp::get_logger("KukaFRIHardwareInterface"), "External control stopped by an error");
}
}  // namespace kuka_sunrise_fri_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_sunrise_fri_driver::KukaFRIHardwareInterface, hardware_interface::SystemInterface)
