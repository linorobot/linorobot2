import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = 'false'
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "urdf/robots", "2wd.urdf.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'rviz', 'description.rviz']
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                {'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])}
            ]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940