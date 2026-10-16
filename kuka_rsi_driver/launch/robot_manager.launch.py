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
from launch_ros.actions import LifecycleNode


def launch_setup(context, *args, **kwargs):
    controller_manager_name = LaunchConfiguration("controller_manager_name")
    robot_model = LaunchConfiguration("robot_model")
    use_gpio = LaunchConfiguration("use_gpio")
    driver_version = LaunchConfiguration("driver_version")
    ns = LaunchConfiguration("namespace")

    # The driver config contains only parameters that can be changed after startup
    driver_config = get_package_share_directory("kuka_rsi_driver") + "/config/driver_config.yaml"

    robot_manager_node = LifecycleNode(
        name=["robot_manager"],
        namespace=ns,
        package="kuka_rsi_driver",
        executable=(
            "robot_manager_node_rsi_only"
            if driver_version.perform(context) == "rsi_only"
            else "robot_manager_node_extended"
        ),
        parameters=[driver_config, {"robot_model": robot_model, "use_gpio": use_gpio, "controller_manager_name": controller_manager_name}],
    )

    return [robot_manager_node]


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("controller_manager_name", default_value="b_controlled_box_cm"))
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="kr6_r700_sixx"))
    launch_arguments.append(
        DeclareLaunchArgument("use_gpio", default_value="false", choices=["true", "false"])
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "driver_version",
            default_value="rsi_only",
            description="Select the driver version to use",
            choices=["rsi_only", "eki_rsi", "mxa_rsi"],
        )
    )
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])