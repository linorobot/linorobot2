# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    zed_sensors = ['zed', 'zed2', 'zed2i', 'zedm']
    zed_common_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'zed_common.yaml']
    )

    oakd_sensors = ['oakd', 'oakdlite', 'oakdpro']
    to_oakd_vars = {
        "oakd": "OAK-D",
        "oakdlite": "OAK-D-LITE",
        "oakdpro": "OAK-D-PRO"
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            name='sensor', 
            default_value='realsense',
            description='Sensor to launch'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
            )),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('sensor'), 'realsense')),
            launch_arguments={
                'pointcloud.enable': 'true',
                'ordered_pc': 'true', 
                'initial_reset': 'true'
            }.items()   
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zed_wrapper'), 'launch/include', 'zed_camera.launch.py']
            )),
            condition=IfCondition(PythonExpression(['"', LaunchConfiguration('sensor'), '" in "', str(zed_sensors), '"'])),
            launch_arguments={
                'camera_model': LaunchConfiguration('sensor'),
                'config_common_path': zed_common_config_path,
                'camera_name': '',
                'node_name': 'zed',
                'publish_urdf': 'true',
                'base_frame': 'camera_link'
            }.items()   
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('depthai_examples'), 'launch', 'stereo.launch.py']
            )),
            condition=IfCondition(PythonExpression(['"', LaunchConfiguration('sensor'), '" in "', str(oakd_sensors), '"'])),
            launch_arguments={
                'camera_model': to_oakd_vars.get(LaunchConfiguration('sensor'), None),              
            }.items()   
        ),
    ])

