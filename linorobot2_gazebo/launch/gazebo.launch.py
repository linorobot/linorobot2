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
from launch.conditions import IfCondition

from nav2_common.launch import ReplaceString


def generate_launch_description():
    use_sim_time = True
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    robot_base = os.getenv('LINOROBOT2_BASE')
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "urdf/robots", f"{robot_base}.urdf.xacro"]
    )
    
    world_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "worlds", "playground.sdf"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='namespace',
            default_value='linorobot2',
            description='Robot Namespace'
        ),
        
        DeclareLaunchArgument(
            name='use_simulator', 
            default_value='true',
            description='Enable Gazebo server'
        ),

        DeclareLaunchArgument(
            name='gui', 
            default_value='true',
            description='Enable Gazebo Client'
        ),
        
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),

        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='odometry',
            description='EKF out odometry topic'
        ),
        
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='0.5',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
            
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn heading'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(LaunchConfiguration('use_simulator')),
            launch_arguments={
                'gz_args': [' -r -s ', LaunchConfiguration('world')]
            }.items()
            
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(LaunchConfiguration('use_simulator')),
            launch_arguments={
                'gz_args': [' -g']
            }.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-name', LaunchConfiguration('namespace'), 
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'config_file': PathJoinSubstitution(
                        [FindPackageShare("linorobot2_gazebo"), "config", "mecanum.yaml"]
                    ),
                    "expand_gz_topic_names": True,
                }
            ],
            # remappings=[
            #     ('camera/camera_info', 'camera/color/camera_info'),
            #     ('camera/image', 'camera/color/image_raw'),
            #     ('camera/depth_image', 'camera/depth/image_rect_raw'),
            #     ('camera/points', 'camera/depth/color/points'),
            # ]
        ),

        Node(
            package='linorobot2_gazebo',
            namespace=LaunchConfiguration('namespace'),
            executable='command_timeout',
            name='command_timeout'
        ),

        

        Node(
            package='robot_localization',
            executable='ekf_node',
            namespace=LaunchConfiguration('namespace'),
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ReplaceString(
                    source_file=ekf_config_path,
                    replacements={"<robot_namespace>": ("/", LaunchConfiguration("namespace"))},
                )
            ],
            remappings=[
                ("odometry/filtered", LaunchConfiguration("odom_topic")),
                ("/tf", "tf"),
                ("/tf_static", "tf_static")
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
                'urdf': LaunchConfiguration('urdf')
            }.items()
        )
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940
