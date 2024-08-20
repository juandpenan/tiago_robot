# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_pal.arg_utils import read_launch_argument
from launch_pal.arg_utils import LaunchArgumentsBase
from launch.actions import DeclareLaunchArgument
from tiago_description.launch_arguments import TiagoArgs
from launch_ros.actions import Node


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    arm_motor_model: DeclareLaunchArgument = TiagoArgs.arm_motor_model


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    launch_description.add_action(OpaqueFunction(function=gravity_compensation_controller))


def gravity_compensation_controller(context, *args, **kwargs):

    # pkg_share_foder = get_package_share_directory('tiago_controller_configuration')
    arm_motor_model = read_launch_argument('arm_motor_model', context)

    param_file = os.path.join(get_package_share_directory(
        'tiago_controller_configuration'), "config", "gravity_compensation_controller_" +
        arm_motor_model + ".yaml")

    gravity_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "gravity_compensation_controller", "--param-file", os.path.join(
                get_package_share_directory('tiago_controller_configuration'),
                'config', param_file), "--inactive"],
    )

    return [gravity_spawner_node]
