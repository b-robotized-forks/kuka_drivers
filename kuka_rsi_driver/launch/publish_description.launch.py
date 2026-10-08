# Copyright 2023 Aron Svastits
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

import json

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def rsi_xml_config_xacro_arg(context, path, inline):
    """Return the xacro argument for rsi_xml_config_file.

    With inline=true, the YAML file at path is read on this machine and passed as compact JSON
    (a YAML flow mapping) instead of the path. The driver parses content starting with '{' directly,
    so the hardware doesn't need the file, e.g. when it runs on a ctrlX controller manager.
    """
    value = path.perform(context)
    if value and inline.perform(context) == "true":
        with open(value, encoding="utf-8") as config_file:
            value = json.dumps(yaml.safe_load(config_file), separators=(",", ":"))
        if "'" in value:
            raise RuntimeError(f"RSI XML config '{path.perform(context)}' must not contain \"'\".")
    # Quoted as one token: the command is split like a shell command line
    return f"rsi_xml_config_file:='{value}'"

def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    mode = LaunchConfiguration("mode")
    use_gpio = LaunchConfiguration("use_gpio")
    driver_version = LaunchConfiguration("driver_version")
    client_ip = LaunchConfiguration("client_ip")
    client_port = LaunchConfiguration("client_port")
    mxa_client_port = LaunchConfiguration("mxa_client_port")
    controller_ip = LaunchConfiguration("controller_ip")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    roundtrip_time = LaunchConfiguration("roundtrip_time")
    verify_robot_model = LaunchConfiguration("verify_robot_model")
    rsi_xml_config_file = LaunchConfiguration("rsi_xml_config_file")
    inline_rsi_xml_config = LaunchConfiguration("inline_rsi_xml_config")
    read_robot_status = LaunchConfiguration("read_robot_status")
    read_cartesian_pose = LaunchConfiguration("read_cartesian_pose")
    read_cartesian_setpoint = LaunchConfiguration("read_cartesian_setpoint")
    is_async = LaunchConfiguration("is_async")
    async_scheduling_policy = LaunchConfiguration("async_scheduling_policy")
    async_thread_priority = LaunchConfiguration("async_thread_priority")
    async_affinity = LaunchConfiguration("async_affinity")
    ns = LaunchConfiguration("namespace")
    
    if ns.perform(context) == "":
        tf_prefix = ""
    else:
        tf_prefix = ns.perform(context) + "_"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(f"kuka_{robot_family.perform(context)}_support"),
                    "urdf",
                    robot_model.perform(context) + ".urdf.xacro",
                ]
            ),
            " ",
            "mode:=",
            mode,
            " ",
            "use_gpio:=",
            use_gpio,
            " ",
            "driver_version:=",
            driver_version,
            " ",
            "client_port:=",
            client_port,
            " ",
            "mxa_client_port:=",
            mxa_client_port,
            " ",
            "client_ip:=",
            client_ip,
            " ",
            "controller_ip:=",
            controller_ip,
            " ",
            "prefix:=",
            tf_prefix,
            " ",
            "x:=",
            x,
            " ",
            "y:=",
            y,
            " ",
            "z:=",
            z,
            " ",
            "roll:=",
            roll,
            " ",
            "pitch:=",
            pitch,
            " ",
            "yaw:=",
            yaw,
            " ",
            "roundtrip_time:=",
            roundtrip_time,
            " ",
            "verify_robot_model:=",
            verify_robot_model,
            " ",
            rsi_xml_config_xacro_arg(context, rsi_xml_config_file, inline_rsi_xml_config),
            " ",
            "read_robot_status:=",
            read_robot_status,
            " ",
            "read_cartesian_pose:=",
            read_cartesian_pose,
            " ",
            "read_cartesian_setpoint:=",
            read_cartesian_setpoint,
            " ",
            "is_async:=",
            is_async,
            " ",
            "async_scheduling_policy:=",
            async_scheduling_policy,
            " ",
            "async_thread_priority:=",
            async_thread_priority,
            " ",
            "async_affinity:=",
            async_affinity,
        ],
        on_stderr="capture",
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        namespace=ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return [robot_state_publisher]


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="kr6_r700_sixx"))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value="agilus"))
    launch_arguments.append(DeclareLaunchArgument("mode", default_value="hardware"))
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
    launch_arguments.append(DeclareLaunchArgument("client_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("client_port", default_value="59152"))
    launch_arguments.append(DeclareLaunchArgument("mxa_client_port", default_value="1337"))
    launch_arguments.append(DeclareLaunchArgument("controller_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("y", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("yaw", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roundtrip_time", default_value="4000"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "verify_robot_model", default_value="true", choices=["true", "false"]
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "rsi_xml_config_file",
            default_value="",
            description=(
                "Absolute path to the RSI XML config YAML, as seen by the machine running the "
                "hardware (on a ctrlX: a path on the ctrlX). Empty = driver defaults."
            ),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "inline_rsi_xml_config",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Read rsi_xml_config_file on this machine and pass its content in the robot "
                "description instead of the path. Needed when the hardware can't read files from "
                "this machine (ctrlX controller manager)."
            ),
        )
    )
    for read_arg, what in (
        ("read_robot_status", "robot_status sensor (program state, speed scaling)"),
        ("read_cartesian_pose", "cartesian_pose sensor (actual TCP pose, RIst)"),
        ("read_cartesian_setpoint", "cartesian_setpoint sensor (setpoint TCP pose, RSol)"),
    ):
        launch_arguments.append(
            DeclareLaunchArgument(
                read_arg,
                default_value="false",
                choices=["true", "false"],
                description=f"Add the {what} to the robot description.",
            )
        )
    launch_arguments.append(
        DeclareLaunchArgument(
            "is_async",
            default_value="false",
            choices=["true", "false"],
            description="Run the hardware component asynchronously to the controller manager loop.",
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "async_scheduling_policy",
            default_value="detached",
            description=(
                "Scheduling policy of the async hardware thread (only used with is_async:=true): "
                "'synchronized' or 'detached' in ros2_control, 'slave' on the ctrlX (b_controlled_box)."
            ),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "async_thread_priority",
            default_value="69",
            description="Priority of the async hardware thread (only used with is_async:=true).",
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "async_affinity",
            default_value="[]",
            description=(
                "CPU cores for the async hardware thread, e.g. '[2,3]' without spaces "
                "(only used with is_async:=true). Empty list = no pinning."
            ),
        )
    )

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])