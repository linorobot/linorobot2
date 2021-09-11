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
            parameters=[ydlidar_config_path]
        ),

        Node(
            condition=LaunchConfigurationEquals("sensor", 'rplidar'),
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        )
    ])

