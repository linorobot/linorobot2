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
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'sensors.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        #run robot driver
        #https://github.com/linorobot/sphero_rvr
        Node(
            package='sphero_rvr',
            executable='sphero_node',
            name='sphero_node',
            output='screen'
        ),
        #https://github.com/christianrauch/raspicam2_node
        Node(
            package='raspicam2',
            executable='raspicam2_node',
            name='raspicam2',
            output='screen'
        ),
        #you can load your custom urdf launcher here
        #for demo's sake we'll use the default description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),
        #hardware/sensor specific launch files
        #for demo's sake we'll use the default description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
        )
    ])