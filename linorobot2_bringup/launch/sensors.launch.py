import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    laser_sensor = os.getenv('LINOROBOT2_LASER_SENSOR', '')
    depth_sensor = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')

    rplidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('rplidar_ros'), 'launch', 'rplidar.launch.py']
    )

    ldlidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('ldlidar'), 'launch', 'ldlidar.launch.py']
    )

    realsense_launch_path = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path),
            condition=IfCondition(PythonExpression(['"rplidar" == "', laser_sensor, '"'])),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_launch_path),
            condition=IfCondition(PythonExpression(['"ldlidar" == "', laser_sensor, '"'])),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            condition=IfCondition(PythonExpression(['"realsense" == "', depth_sensor, '"'])),
            launch_arguments={
                'filters': 'pointcloud',
                'ordered_pc': 'true'
            }.items()   
        )
    ])

