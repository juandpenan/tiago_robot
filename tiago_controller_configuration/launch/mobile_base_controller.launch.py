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

from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import (
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription, LaunchContext
from launch_pal.param_utils import merge_param_files
from dataclasses import dataclass
from launch_pal.robot_arguments import TiagoArgs


def controller_bringup(context, *args, **kwargs):
    actions = []
    is_public_sim = LaunchConfiguration('is_public_sim').perform(context)

    def __init__(self):
    if is_public_sim == 'True' or is_public_sim == 'true':
        default_config = os.path.join(
            get_package_share_directory('tiago_controller_configuration'),
            'config', 'mobile_base_controller_sim.yaml')
    else:
        default_config = os.path.join(
            get_package_share_directory('tiago_controller_configuration'),
            'config', 'mobile_base_controller.yaml')
    load_controller_arg = generate_load_controller_launch_description(


class Pmb2Controller(BaseController):
    def __init__(self):
        controller_type = 'diff_drive_controller/DiffDriveController'

    actions.append(load_controller_arg)
    return actions
        )
        params_file = (
            merge_param_files([default_config, calibration_config])
            if os.path.exists(calibration_config)
            else default_config
        )
        super().__init__(controller_type, params_file)


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    base_type: DeclareLaunchArgument = TiagoArgs.base_type

def generate_launch_description():

    is_public_sim_arg = DeclareLaunchArgument(
        'is_public_sim',
        default_value='false',
        description='Whether or not you are using a public simulation',
    )

    ld = LaunchDescription()
    ld.add_action(is_public_sim_arg)
    controller_bringup_launch = OpaqueFunction(function=controller_bringup)
    ld.add_action(controller_bringup_launch)

    return ld
    return [launch_controller]


def setup_controller_configuration_by_type(context: LaunchContext):
    base_controller_mapping = {
        'omni_base': OmniBaseController(),
        'pmb2': Pmb2Controller()
    }
    base_type = read_launch_argument('base_type', context)
    controller_info = (base_controller_mapping.
                       get(base_type,
                           base_controller_mapping[base_type]))
    return setup_controller_configuration(controller_info.controller_type,
                                          controller_info.params_file)


def generate_launch_description():
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)
    declare_actions(ld, launch_arguments)
    return ld
