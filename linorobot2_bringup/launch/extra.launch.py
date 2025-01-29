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

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    laser_filter_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'box_laser_filter.yaml']
    )

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'lasers.launch.py']
    )

    return LaunchDescription([
        Node(
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB1'},
                {'topic_name': '/base/scan/unfiltered'},
                {'lidar_frame': 'base_laser'},
                {'range_threshold': 0.005}
            ]
        ),
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                laser_filter_config_path
            ],
            remappings=[
                ('/scan', '/base/scan/unfiltered'),
                ('/scan_filtered', '/base/scan')
            ]
        )
    ])

