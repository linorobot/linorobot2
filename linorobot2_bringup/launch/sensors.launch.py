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
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    laser_sensor_name = os.getenv('LINOROBOT2_LASER_SENSOR', '')
    depth_sensor_name = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')
    
    fake_laser_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'fake_laser.yaml']
    )

    zed_common_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'zed_common.yaml']
    )

    zed_camera_config_path = PathJoinSubstitution(
        [FindPackageShare('zed_wrapper'), 'config', f'{depth_sensor_name}.yaml']
    )

    zed_xacro_path = PathJoinSubstitution(
        [FindPackageShare('zed_wrapper'), 'urdf', 'zed_descr.urdf.xacro']
    )

    zed_launch_arguments={
        'camera_model': depth_sensor_name,
        'camera_name': '',
        'node_name': 'zed',
        'config_common_path': zed_common_config_path,
        'config_camera_path': zed_camera_config_path,
        'publish_urdf': 'true',
        'xacro_path': zed_xacro_path,
        'base_frame': 'camera_link'
    }

    #indices
    #0 - package name (str)
    #1 - launch folder (str)
    #2 - launch file (str)
    #3 - launch arguments (dict)
    #4 - depth topic (str)
    #5 - depth info topic (str)
    depth_sensors = {
        '': ['', '', '', {}, '', ''],
        'realsense': ['realsense2_camera', 'launch', 'rs_launch.py', {'filters': 'pointcloud','ordered_pc': 'true'}, '/camera/depth/image_rect_raw', '/camera/depth/camera_info'],
        'astra': ['astra_camera', 'launch', 'astra_launch.py', {}, '/depth/rgb/ir', '/camera_info'],
        'zed': ['zed_wrapper', 'launch/include', 'zed_camera.launch.py', zed_launch_arguments, '/zed/depth/depth_registered', '/zed/depth/camera_info'],
        'zed2': ['zed_wrapper', 'launch/include', 'zed_camera.launch.py', zed_launch_arguments, '/zed/depth/depth_registered', '/zed/depth/camera_info'],
        'zed2i': ['zed_wrapper', 'launch/include', 'zed_camera.launch.py', zed_launch_arguments, '/zed/depth/depth_registered', '/zed/depth/camera_info'],
        'zedm': ['zed_wrapper', 'launch/include', 'zed_camera.launch.py', zed_launch_arguments, '/zed/depth/depth_registered', '/zed/depth/camera_info']
    }

    laser_sensors = {
        '': ['', '', '', {}],
        'rplidar': ['linorobot2_bringup', 'launch', 'vendors.launch.py', {'sensor': 'rplidar'}],
        'ydlidar': ['linorobot2_bringup', 'launch', 'vendors.launch.py', {'sensor': 'ydlidar'}],
        'ldlidar': ['ldlidar', 'launch', 'ldlidar.launch.py', {'serial_port': '/dev/ttyUSB0'}]
    }

    laser_sensors.update(depth_sensors) #make depth sensors available as laser sensors as well

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare(laser_sensors[laser_sensor_name][0]), laser_sensors[laser_sensor_name][1], laser_sensors[laser_sensor_name][2]]
    )

    depth_launch_path = PathJoinSubstitution(
        [FindPackageShare(depth_sensors[depth_sensor_name][0]), depth_sensors[depth_sensor_name][1], depth_sensors[depth_sensor_name][2]]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path),
            condition=IfCondition(PythonExpression(['"" != "', laser_sensor_name, '" and "', depth_sensor_name, '" != "', laser_sensor_name, '"'])),
            launch_arguments=laser_sensors[laser_sensor_name][3].items()   
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depth_launch_path),
            condition=IfCondition(PythonExpression(['"" != "', depth_sensor_name, '"'])),
            launch_arguments=depth_sensors[depth_sensor_name][3].items()   
        ),

        Node(
            condition=IfCondition(PythonExpression(['"" != "', laser_sensor_name, '" and ', '"', laser_sensor_name, '" in "', str(list(depth_sensors.keys())[1:]), '"'])),
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            remappings=[('depth', depth_sensors[depth_sensor_name][4]),
                        ('depth_camera_info', depth_sensors[depth_sensor_name][5])],
            parameters=[fake_laser_config_path]
        )
    ])

