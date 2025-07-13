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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def launch_rplidar(context, *args, **kwargs):
    lidar_str = context.perform_substitution(LaunchConfiguration('sensor'))
    rplidar_sensors = [
        'a1',
        'a2',
        'a3',
        'c1',
        's1',
        's2',
        's3',
    ]
    
    if lidar_str in rplidar_sensors:
        launch_file = f'sllidar_{lidar_str}_launch.py'
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('sllidar_ros2'), 'launch', launch_file]
            )),
            launch_arguments={
                'serial_port': '/dev/rplidar', 
                'frame_id': LaunchConfiguration('frame_id'),
            }.items()   
        )]
    else:
        return []

def generate_launch_description():
    lidar_options =[
        'rplidar', 
        'ldlidar',
        'ld06',
        'ld19',
        'stl27l',
        'ydlidar',
        'xv11',
        'a1',
        'a2',
        'a3',
        'c1',
        's1',
        's2',
        's3',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sensor', 
            default_value='ydlidar',
            description='Sensor to launch',
            choices=lidar_options
        ),

        DeclareLaunchArgument(
            name='topic_name', 
            default_value='scan',
            description='Laser Topic Name'
        ),

        DeclareLaunchArgument(
            name='frame_id', 
            default_value='laser',
            description='Laser Frame ID'
        ),

        DeclareLaunchArgument(
            name='lidar_transport',
            default_value='serial',
            description='Lidar transport: serial, udp_server, udp_client, tcp_server, tcp_client'
        ),

        DeclareLaunchArgument(
            name='lidar_serial_port',
            default_value='/dev/ttyUSB0',
            description='Lidar serial port device name'
        ),

        DeclareLaunchArgument(
            name='lidar_server_ip',
            default_value='0.0.0.0',
            description='Lidar server ip'
        ),

        DeclareLaunchArgument(
            name='lidar_server_port',
            default_value='8889',
            description='Lidar server port number'
        ),

        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('sensor'), 'ydlidar')),
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            remappings=[('scan', LaunchConfiguration('topic_name'))],
            parameters=[{ 
                'port': '/dev/ydlidar',
                'frame_id': LaunchConfiguration('frame_id'),
                'ignore_array': '',
                'baudrate': 128000,
                'lidar_type': 1,
                'device_type': 6,
                'sample_rate': 5,
                'abnormal_check_count': 4,
                'fixed_resolution': True,
                'reversion': False,
                'inverted': True,
                'auto_reconnect': True,
                'isSingleChannel': False,
                'intensity': False,
                'support_motor_dtr': True,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 10.0,
                'range_min': 0.12,
                'frequency': 5.0,
                'invalid_range_is_inf': False
            }]
        ),

        # Node(
        #     condition=IfCondition(EqualsSubstitution(LaunchConfiguration('sensor'), 'rplidar')),
        #     name='rplidar_composition',
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     output='screen',
        #     remappings=[('scan', LaunchConfiguration('topic_name'))],
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'serial_baudrate': 115200,  # A1 / A2
        #         'frame_id': LaunchConfiguration('frame_id'),
        #         'inverted': False,
        #         'angle_compensate': True,
        #     }],
        # ),

        Node( 
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('sensor'), 'xv11')),
            name='xv_11_driver',
            package='xv_11_driver',
            executable='xv_11_driver',
            output='screen',
            remappings=[('scan', LaunchConfiguration('topic_name'))],
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud_rate': 115200, 
                'frame_id': LaunchConfiguration('frame_id'),
                'firmware_version': 2
            }],
        ),

        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('sensor'), 'ldlidar')),
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'topic_name': LaunchConfiguration('topic_name')},
                {'lidar_frame': LaunchConfiguration('frame_id')},
                {'range_threshold': 0.005}
            ]
        ),

        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('sensor'), 'ld06')),
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ld06',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD06'},
                {'topic_name': LaunchConfiguration('topic_name')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'comm_mode': LaunchConfiguration('lidar_transport')},
                {'port_name': LaunchConfiguration('lidar_serial_port')},
                {'port_baudrate': 230400},
                {'server_ip': LaunchConfiguration('lidar_server_ip')},
                {'server_port': LaunchConfiguration('lidar_server_port')},
                {'laser_scan_dir': True},
                {'bins': 456},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}
            ]
        ),

        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('sensor'), 'ld19')),
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ld19',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD19'},
                {'topic_name': LaunchConfiguration('topic_name')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'comm_mode': LaunchConfiguration('lidar_transport')},
                {'port_name': LaunchConfiguration('lidar_serial_port')},
                {'port_baudrate': 230400},
                {'server_ip': LaunchConfiguration('lidar_server_ip')},
                {'server_port': LaunchConfiguration('lidar_server_port')},
                {'laser_scan_dir': True},
                {'bins': 456},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}
            ]
        ),

        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('sensor'), 'stl27l')),
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='stl27l',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_STL27L'},
                {'topic_name': LaunchConfiguration('topic_name')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'comm_mode': LaunchConfiguration('lidar_transport')},
                {'port_name': LaunchConfiguration('lidar_serial_port')},
                {'port_baudrate': 921600},
                {'server_ip': LaunchConfiguration('lidar_server_ip')},
                {'server_port': LaunchConfiguration('lidar_server_port')},
                {'laser_scan_dir': True},
                {'bins': 2160},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}
            ]
        ),
        OpaqueFunction(function=launch_rplidar)
    ])

