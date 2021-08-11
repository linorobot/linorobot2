import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    rplidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('rplidar_ros'), 'launch', 'rplidar.launch.py']
    )

    ldlidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('ldlidar'), 'launch', 'ldlidar.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='laser_sensor', 
            default_value='',
            description='Laser Sensor'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path),
            condition=LaunchConfigurationEquals("laser_sensor", 'rplidar')
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_launch_path),
            condition=LaunchConfigurationEquals("laser_sensor", 'ldlidar')
        )
    ])