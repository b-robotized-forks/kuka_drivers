# Copyright 2023 KUKA Hungaria Kft.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_gpio = LaunchConfiguration("use_gpio")
    driver_version = LaunchConfiguration("driver_version")
    ns = LaunchConfiguration("namespace")
    jtc_config = LaunchConfiguration("jtc_config")
    gpio_config = LaunchConfiguration("gpio_config")
    controller_manager_name = LaunchConfiguration("controller_manager_name")

    # Spawn controllers
    def controller_spawner(controller_name, param_file=None, activate=False):
        arg_list = [
            controller_name,
            "-c",
            controller_manager_name,
            "-n",
            ns,
            "-p",
            "scenario_controllers.yaml"
        ]

        # Add param-file if it's provided
        # if param_file:
        #     arg_list.extend(["--param-file", param_file])

        if not activate:
            arg_list.append("--inactive")

        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=arg_list,
        )

    controllers = {
        "joint_state_broadcaster": None,
        "joint_trajectory_controller": jtc_config,
        "event_broadcaster": None,
    }

    if use_gpio.perform(context) == "true":
        controllers["gpio_controller"] = gpio_config

    if driver_version.perform(context) in {"eki_rsi", "mxa_rsi"}:
        controllers["control_mode_handler"] = None
        controllers["kss_message_handler"] = None

    controller_spawners = [
        controller_spawner(name, param_file)
        for name, param_file in controllers.items()
    ]

    return controller_spawners


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(
        DeclareLaunchArgument("use_gpio", default_value="false", choices=["true", "false"])
    )
    launch_arguments.append(DeclareLaunchArgument("controller_manager_name", default_value="b_controlled_box_cm"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "driver_version",
            default_value="rsi_only",
            description="Select the driver version to use",
            choices=["rsi_only", "eki_rsi", "mxa_rsi"],
        )
    )
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(
        DeclareLaunchArgument(
            "jtc_config",
            default_value=get_package_share_directory("kuka_rsi_driver")
            + "/config/joint_trajectory_controller_config.yaml",
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "gpio_config",
            default_value=get_package_share_directory("kuka_rsi_driver")
            + "/config/gpio_controller_config.yaml",
        )
    )

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])