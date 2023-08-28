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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "worlds", "playground.sdf"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'gz_args': world_path
            }.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description', '-name', "linorobot2",],
        ),

        Node(
            package='linorobot2_gazebo',
            executable='command_timeout.py',
            name='command_timeout'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
        )
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940