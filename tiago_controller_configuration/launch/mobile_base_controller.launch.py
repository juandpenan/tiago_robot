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
    generate_load_controller_launch_description)
from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription, LaunchContext
from launch_pal.param_utils import merge_param_files
from dataclasses import dataclass
from launch_pal.robot_arguments import TiagoArgs


class BaseController:
    def __init__(self, controller_type, params_file):
        self.controller_type = controller_type
        self.params_file = params_file

    def get_controller_info(self):
        return self.controller_type, self.params_file


class Omni(BaseController):
    def __init__(self, is_public_sim):
        controller_type = 'omni_drive_controller/OmniDriveController'
        params_file = os.path.join(
            get_package_share_directory('omni_base_controller_configuration'),
            'config', 'mobile_base_controller.yaml'
        )
        super().__init__(controller_type, params_file)


class Pmb2(BaseController):
    def __init__(self, is_public_sim):
        controller_type = 'diff_drive_controller/DiffDriveController'
        if is_public_sim == 'True' or is_public_sim == 'true':
            default_config = os.path.join(
                get_package_share_directory('tiago_controller_configuration'),
                'config', 'mobile_base_controller_sim.yaml')
        else:
            default_config = os.path.join(
                get_package_share_directory('tiago_controller_configuration'),
                'config', 'mobile_base_controller.yaml')

        calibration_config = '/etc/calibration/master_calibration.yaml'

        if os.path.exists(calibration_config):
            params_file = merge_param_files([default_config,
                                             calibration_config])
        else:
            params_file = default_config

        super().__init__(controller_type, params_file)


class ControllerFactory:

    CONTROLLER_MAPPING = {
        'omni_base': Omni,
        'pmb2': Pmb2,
    }

    @staticmethod
    def create_controller(base_type: str, is_public_sim: str):
        controller_class = ControllerFactory.CONTROLLER_MAPPING.get(base_type)
        if (controller_class):
            return controller_class(is_public_sim)
        else:
            raise ValueError(f"Unknown base type: {base_type}")


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    base_type: DeclareLaunchArgument = TiagoArgs.base_type
    is_public_sim = DeclareLaunchArgument(
        'is_public_sim',
        default_value='False',
        description='Whether or not you are using a public simulation')


def declare_actions(launch_description: LaunchDescription,
                    launch_args: LaunchArguments):
    launch_description.add_action(
        OpaqueFunction(function=setup_controller_configuration_by_type))


def setup_controller_configuration(controller_type: str,
                                   params_file: str):

    launch_controller = generate_load_controller_launch_description(
        controller_name='mobile_base_controller',
        controller_type=controller_type,
        controller_params_file=params_file
    )
    return [launch_controller]


def setup_controller_configuration_by_type(context: LaunchContext):
    base_type = read_launch_argument('base_type', context)
    is_public_sim = read_launch_argument('is_public_sim', context)

    controller = ControllerFactory.create_controller(base_type,
                                                     is_public_sim)
    controller_info = controller.get_controller_info()
    controller_type, params_file = controller_info
    return setup_controller_configuration(controller_type,
                                          params_file)


def generate_launch_description():
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)
    declare_actions(ld, launch_arguments)
    return ld
