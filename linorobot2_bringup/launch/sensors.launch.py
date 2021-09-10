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
        [FindPackageShare("linorobot2_bringup"), "config", "fake_laser.yaml"]
    )

    #indices
    #0 - package name (str)
    #1 - launch file (str)
    #2 - launch arguments (dict)
    #3 - depth topic (str)
    #4 - depth info topic (str)
    depth_sensors = {
        '': ['', '', {}, '', ''],
        'realsense': ['realsense2_camera', 'rs_launch.py', {'filters': 'pointcloud','ordered_pc': 'true'}, '/camera/depth/image_rect_raw', '/camera/depth/camera_info'],
        'astra': ['astra_camera', 'astra_launch.py', {}, '/depth/rgb/ir', '/camera_info']
    }

    laser_sensors = {
        '': ['', '', {}],
        'rplidar': ['rplidar_ros', 'rplidar.launch.py', {}],
        'ldlidar': ['ldlidar', 'ldlidar.launch.py', {'serial_port': '/dev/ttyUSB0'}],
        'ydlidar': ['linorobot2_bringup', 'vendors.launch.py', {'sensor': 'ydlidar'}],
    }
    laser_sensors.update(depth_sensors) #make depth sensors available as laser sensors as well

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare(laser_sensors[laser_sensor_name][0]), 'launch', laser_sensors[laser_sensor_name][1]]
    )

    depth_launch_path = PathJoinSubstitution(
        [FindPackageShare(depth_sensors[depth_sensor_name][0]), 'launch', depth_sensors[depth_sensor_name][1]]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path),
            condition=IfCondition(PythonExpression(['"" != "', laser_sensor_name, '"'])),
            launch_arguments=laser_sensors[laser_sensor_name][2].items()   
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depth_launch_path),
            condition=IfCondition(PythonExpression(['"" != "', depth_sensor_name, '"'])),
            launch_arguments=depth_sensors[depth_sensor_name][2].items()   
        ),

        Node(
            condition=IfCondition(PythonExpression(['"" != "', laser_sensor_name, '" and ', '"', laser_sensor_name, '" in "', str(list(depth_sensors.keys())[1:]), '"'])),
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            remappings=[('depth', depth_sensors[depth_sensor_name][3]),
                        ('depth_camera_info', depth_sensors[depth_sensor_name][4])],
            parameters=[fake_laser_config_path]
        )
    ])

