# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationNotEquals
from tiago_description.tiago_launch_utils import get_tiago_hw_suffix
from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from launch_pal.robot_arguments import TiagoArgs

from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    arm_type: DeclareLaunchArgument = TiagoArgs.arm_type
    end_effector: DeclareLaunchArgument = TiagoArgs.end_effector
    ft_sensor: DeclareLaunchArgument = TiagoArgs.ft_sensor

    cmd_vel: DeclareLaunchArgument = DeclareLaunchArgument(
        name="cmd_vel",
        default_value="input_joy/cmd_vel",
        description="Joystick cmd_vel topic",
    )


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    launch_description.add_action(OpaqueFunction(function=create_joy_teleop_filename))

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[LaunchConfiguration("teleop_config")],
        remappings=[("cmd_vel", LaunchConfiguration("cmd_vel"))],
    )

    launch_description.add_action(joy_teleop_node)

    pkg_dir = get_package_share_directory("tiago_bringup")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(pkg_dir, "config", "joy_teleop", "joy_config.yaml")],
    )

    launch_description.add_action(joy_node)

    torso_incrementer_server = Node(
        package="joy_teleop",
        executable="incrementer_server",
        name="incrementer",
        namespace="torso_controller",
    )

    launch_description.add_action(torso_incrementer_server)

    head_incrementer_server = Node(
        package="joy_teleop",
        executable="incrementer_server",
        name="incrementer",
        namespace="head_controller",
    )

    launch_description.add_action(head_incrementer_server)

    gripper_incrementer_server = Node(
        package="joy_teleop",
        executable="incrementer_server",
        name="incrementer",
        namespace="gripper_controller",
        condition=LaunchConfigurationNotEquals("end_effector", "no-end-effector"),
    )

    launch_description.add_action(gripper_incrementer_server)

    return


def create_joy_teleop_filename(context):

    hw_suffix = get_tiago_hw_suffix(
        arm=read_launch_argument("arm_type", context),
        end_effector=read_launch_argument("end_effector", context),
        ft_sensor=read_launch_argument("ft_sensor", context),
    )

    joy_teleop_file = f"joy_teleop{hw_suffix}.yaml"

    joy_teleop_path = os.path.join(
        get_package_share_directory("tiago_bringup"),
        "config",
        "joy_teleop",
        joy_teleop_file,
    )

    joy_teleop_config = SetLaunchConfiguration("teleop_config", joy_teleop_path)
    return [joy_teleop_config]
