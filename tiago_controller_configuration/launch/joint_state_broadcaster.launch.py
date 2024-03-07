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
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription, LaunchContext
from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    base_type: DeclareLaunchArgument = DeclareLaunchArgument(
        name='base_type',
        default_value='pmb2',
        choices=['pmb2', 'omni_base'],
        description='Type of base_type [pmb2, omni_base]')


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(function=setup_controller_configuration))
    return


def setup_controller_configuration(context: LaunchContext):

    actions = []
    base_type = read_launch_argument('base_type', context)
    if base_type == 'omni_base':
        launch_controller = generate_load_controller_launch_description(
            controller_name='joint_state_broadcaster',
            controller_type='joint_state_broadcaster/JointStateBroadcaster',
            controller_params_file=os.path.join(
                get_package_share_directory('omni_base_controller_configuration'),
                'config', 'joint_state_broadcaster.yaml'))
        actions.append(launch_controller)
    else:
        launch_controller = generate_load_controller_launch_description(
            controller_name='joint_state_broadcaster',
            controller_type='joint_state_broadcaster/JointStateBroadcaster',
            controller_params_file=os.path.join(
                get_package_share_directory('tiago_controller_configuration'),
                'config', 'joint_state_broadcaster.yaml')
        )
        actions.append(launch_controller)

    return actions


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
