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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    # world_path = PathJoinSubstitution(
    #     [FindPackageShare("aws-robomaker-small-warehouse-world"), "worlds", "small_warehouse", "small_warehouse.world"]
    # )

    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('aws_robomaker_small_warehouse_world'), "/launch", "/small_warehouse.launch.py"])
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )
    ld = LaunchDescription()

    ld.add_action(warehouse_world_cmd)

    ld.add_action(
        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='-1.0',
            description='Robot spawn position in X axis'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='0.0',
            description='Robot spawn position in Y axis'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.0',
            description='Robot spawn position in Z axis'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn heading'
        )
    )        
    
    ld.add_action(
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        )
    )

    ld.add_action(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-entity', 'linorobot2', 
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        )
    )
    ld.add_action(
        Node(
            package='linorobot2_gazebo',
            executable='command_timeout.py',
            name='command_timeout'
        )
    )
    ld.add_action(
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
        )
    )    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
            }.items()
        )
    )  
    return ld

    # description_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    # )

    # return LaunchDescription([
    #     DeclareLaunchArgument(
    #         name='world', 
    #         default_value=world_path,
    #         description='Gazebo world'
    #     ),

    #     DeclareLaunchArgument(
    #         name='spawn_x', 
    #         default_value='0.0',
    #         description='Robot spawn position in X axis'
    #     ),

    #     DeclareLaunchArgument(
    #         name='spawn_y', 
    #         default_value='0.0',
    #         description='Robot spawn position in Y axis'
    #     ),

    #     DeclareLaunchArgument(
    #         name='spawn_z', 
    #         default_value='0.0',
    #         description='Robot spawn position in Z axis'
    #     ),
            
    #     DeclareLaunchArgument(
    #         name='spawn_yaw', 
    #         default_value='0.0',
    #         description='Robot spawn heading'
    #     ),

    #     ExecuteProcess(
    #         cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
    #         output='screen'
    #     ),

    #     Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         name='urdf_spawner',
    #         output='screen',
    #         arguments=[
    #             '-topic', 'robot_description', 
    #             '-entity', 'linorobot2', 
    #             '-x', LaunchConfiguration('spawn_x'),
    #             '-y', LaunchConfiguration('spawn_y'),
    #             '-z', LaunchConfiguration('spawn_z'),
    #             '-Y', LaunchConfiguration('spawn_yaw'),
    #         ]
    #     ),

    #     Node(
    #         package='linorobot2_gazebo',
    #         executable='command_timeout.py',
    #         name='command_timeout'
    #     ),

    #     Node(
    #         package='robot_localization',
    #         executable='ekf_node',
    #         name='ekf_filter_node',
    #         output='screen',
    #         parameters=[
    #             {'use_sim_time': use_sim_time}, 
    #             ekf_config_path
    #         ],
    #         remappings=[("odometry/filtered", "odom")]
    #     ),

    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(description_launch_path),
    #         launch_arguments={
    #             'use_sim_time': str(use_sim_time),
    #             'publish_joints': 'false',
    #         }.items()
    #     )
    # ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940
