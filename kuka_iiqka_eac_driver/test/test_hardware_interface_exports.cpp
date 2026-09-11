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

// Regression test for the deprecated-Handle-API migration: the exported state/command
// interface *names* must exactly match what the old raw-pointer export_state_interfaces()/
// export_command_interfaces() used to export, one-for-one. Nothing here exercises hardware or
// the network SDK - only on_init() + export via the hardware_interface::System wrapper (which
// runs the same old-export-empty -> on_export_state_interfaces() fallback the real
// resource_manager uses).

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_iiqka_eac_driver/hardware_interface.hpp"

namespace
{
hardware_interface::HardwareInfo build_test_hardware_info(size_t joint_count)
{
  hardware_interface::HardwareInfo info;
  // Required for HardwareComponentInterface::on_init() to parse info.joints into its internal
  // joint_state_interfaces_/joint_command_interfaces_ maps at all - without this, the driver
  // still succeeds on_init() (it doesn't look at info.type itself), but no interfaces beyond
  // the "unlisted" ones would be auto-exported.
  info.type = "system";
  info.hardware_parameters["controller_ip"] = "127.0.0.1";
  info.hardware_parameters["client_ip"] = "127.0.0.1";

  auto make_interface = [](const std::string & name)
  {
    hardware_interface::InterfaceInfo iface{};
    iface.name = name;
    return iface;
  };

  for (size_t i = 1; i <= joint_count; ++i)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(i);
    joint.command_interfaces = {
      make_interface(hardware_interface::HW_IF_POSITION),
      make_interface(hardware_interface::HW_IF_STIFFNESS),
      make_interface(hardware_interface::HW_IF_DAMPING),
      make_interface(hardware_interface::HW_IF_EFFORT),
    };
    joint.state_interfaces = {
      make_interface(hardware_interface::HW_IF_POSITION),
      make_interface(hardware_interface::HW_IF_EFFORT),
      make_interface(hardware_interface::HW_IF_COMMANDED_POSITION),
    };
    info.joints.push_back(joint);
  }
  return info;
}

class KukaEACHardwareInterfaceExportTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_kuka_eac_hardware_interface_exports");

    hardware_interface::HardwareComponentParams params;
    params.hardware_info = build_test_hardware_info(kJointCount);
    params.clock = node_->get_clock();
    params.logger = node_->get_logger();

    hw_ = std::make_unique<hardware_interface::System>(
      std::make_unique<kuka_eac::KukaEACHardwareInterface>());
    const auto state = hw_->initialize(params);
    ASSERT_EQ(
      state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
      << "on_init() did not succeed - check the test HardwareInfo matches what "
         "KukaEACHardwareInterface::on_init() validates.";
  }

  static constexpr size_t kJointCount = 6;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<hardware_interface::System> hw_;
};

std::vector<std::string> get_names(
  const std::vector<hardware_interface::StateInterface::ConstSharedPtr> & interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto & interface : interfaces)
  {
    names.push_back(interface->get_name());
  }
  return names;
}

std::vector<std::string> get_names(
  const std::vector<hardware_interface::CommandInterface::SharedPtr> & interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto & interface : interfaces)
  {
    names.push_back(interface->get_name());
  }
  return names;
}
}  // namespace

// Interfaces exported by the removed raw-pointer export_state_interfaces(), for comparison:
//   per joint: <joint>/position, <joint>/effort, <joint>/commanded_position
//   plus:      state/server_state
TEST_F(KukaEACHardwareInterfaceExportTest, ExportsExpectedStateInterfaces)
{
  const auto exported_names = get_names(hw_->export_state_interfaces());

  std::vector<std::string> expected_names;
  for (size_t i = 1; i <= kJointCount; ++i)
  {
    const std::string joint = "joint_" + std::to_string(i);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_COMMANDED_POSITION);
  }
  expected_names.push_back(
    std::string(hardware_interface::STATE_PREFIX) + "/" + hardware_interface::SERVER_STATE);

  EXPECT_THAT(exported_names, ::testing::UnorderedElementsAreArray(expected_names));
}

// Interfaces exported by the removed raw-pointer export_command_interfaces(), for comparison:
//   per joint: <joint>/position, <joint>/effort, <joint>/stiffness, <joint>/damping
//   plus:      runtime_config/control_mode
TEST_F(KukaEACHardwareInterfaceExportTest, ExportsExpectedCommandInterfaces)
{
  const auto exported_names = get_names(hw_->export_command_interfaces());

  std::vector<std::string> expected_names;
  for (size_t i = 1; i <= kJointCount; ++i)
  {
    const std::string joint = "joint_" + std::to_string(i);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_STIFFNESS);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_DAMPING);
  }
  expected_names.push_back(
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::CONTROL_MODE);

  EXPECT_THAT(exported_names, ::testing::UnorderedElementsAreArray(expected_names));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
