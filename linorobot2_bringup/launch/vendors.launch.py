from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    ydlidar_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_bringup"), "config", "ydlidar.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sensor', 
            default_value='ydlidar',
            description='Sensor to launch'
        ),

        Node(
            condition=LaunchConfigurationEquals("sensor", 'ydlidar'),
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[ydlidar_config_path],
            node_namespace='/',
        )
    ])

