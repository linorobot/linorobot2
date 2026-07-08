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

# Single-terminal mapping stack (lidar + IMU SLAM, no wheel encoders).
#
# This is the launch-file version of slam_imu.sh, so the whole mapping stack
# starts (and stops, on one Ctrl-C) with a single command:
#
#     ros2 launch linorobot2_bringup robot.launch.py
#
# Keep teleop in a SECOND terminal -- teleop_keyboard.py reads raw keystrokes
# and needs its own TTY, so it cannot share this terminal:
#
#     python3 teleop_keyboard.py
#
# See the live map from your laptop's browser (no rviz, no third terminal): this
# launch also serves it on port 8000, so open  http://<robot-ip>:8000  (or tunnel
# with  ssh -L 8000:localhost:8000 <user>@<robot-ip>  and open localhost:8000).
#
# Pipeline:
#   RPLIDAR A3 --360 deg--> /scan_raw
#   angle_laser_filter --keep +/-90 deg--> /scan   (drops the rear; battery)
#   rf2o_laser_odometry --(odom->base_footprint)--> motion from scan matching
#   mpu6050_imu --> /imu/data                       (published, not owning TF)
#   slam_toolbox --(map->odom)--> builds the map
#   micro_ros_agent <--serial--> Teensy 4.1         (bridges /cmd_vel to motors)
#   map_viewer.py --> http://<robot-ip>:8000        (live map as a web page)
#
# The micro-ROS agent is what lets teleop actually move the robot: teleop_keyboard.py
# publishes /cmd_vel, the agent forwards it over serial to the Teensy firmware
# (controlCallback in main.cpp), which drives the AK10-9 motors. The Teensy only
# publishes odom/unfiltered as a TOPIC (not a TF), so it does not fight rf2o for
# odom->base_footprint. Set micro_ros:=false if you push the robot by hand instead.
#
# WHY rf2o: with no wheel encoders there is no odometry, so slam_toolbox would
# never see the minimum_travel_distance needed to add a scan. rf2o derives real
# x/y/yaw by matching consecutive (now 180 deg) scans and owns odom->base_footprint.
# The IMU only publishes /imu/data (publish_odom_tf:=false) so the two do not fight.
#
# Do NOT run this alongside the stock linorobot2 bringup -- its EKF also owns
# odom->base_footprint and the two would conflict.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    angle_filter_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'angle_laser_filter.yaml']
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'rviz', 'slam_imu.rviz']
    )
    default_slam_params = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'config', 'slam.yaml']
    )

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='lidar_port',
            default_value='/dev/rplidar',
            description='RPLIDAR serial port (udev symlink preferred)'
        ),
        DeclareLaunchArgument(
            name='lidar_baudrate',
            default_value='256000',
            description='RPLIDAR A3 serial baudrate'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Open rviz2 (leave false over SSH / headless)'
        ),
        DeclareLaunchArgument(
            name='micro_ros',
            default_value='true',
            description='Start the micro-ROS agent so teleop /cmd_vel drives the motors'
        ),
        DeclareLaunchArgument(
            name='base_serial_port',
            default_value='/dev/ttyACM0',
            description='Teensy micro-ROS serial port'
        ),
        DeclareLaunchArgument(
            name='micro_ros_baudrate',
            default_value='921600',
            description='micro-ROS serial baudrate (must match the Teensy firmware)'
        ),
        DeclareLaunchArgument(
            name='map_viewer',
            default_value='true',
            description='Serve the live map as a web page on port 8000 (view from a browser)'
        ),
        DeclareLaunchArgument(
            name='map_viewer_path',
            default_value='/home/jetson1/Desktop/map_viewer.py',
            description='Path to the map_viewer.py web viewer script'
        ),
        DeclareLaunchArgument(
            name='slam',
            default_value='true',
            description='Start slam_toolbox (mapping). Set false when running '
                        'navigation, where AMCL owns map->odom localization.'
        ),
        DeclareLaunchArgument(
            name='slam_params_file',
            default_value=default_slam_params,
            description="linorobot2's tuned slam_toolbox config (faster map updates "
                        'than slam_toolbox defaults). Override to use your own.'
        ),

        # micro-ROS agent: bridges the Teensy firmware to ROS 2 over serial, so
        # teleop's /cmd_vel reaches the motors (and odom/unfiltered + /imu come back).
        Node(
            condition=IfCondition(LaunchConfiguration('micro_ros')),
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration('base_serial_port'),
                       '--baudrate', LaunchConfiguration('micro_ros_baudrate')],
        ),

        # RPLIDAR A3: the driver publishes the full 360 deg scan, so send it out
        # on scan_raw and crop the rear with the angular bounds filter below.
        GroupAction([
            SetRemap(src='scan', dst='scan_raw'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a3_launch.py']
                )),
                launch_arguments={
                    'serial_port': lidar_port,
                    'serial_baudrate': lidar_baudrate,
                    'frame_id': 'laser',
                }.items()
            ),
        ]),

        # Crop scan_raw to the front 180 deg (+/-90 deg) and republish as /scan.
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='angle_laser_filter',
            output='screen',
            parameters=[angle_filter_config_path],
            remappings=[
                ('scan', 'scan_raw'),
                ('scan_filtered', 'scan'),
            ],
        ),

        # IMU: publishes /imu/data only. rf2o owns odom->base_footprint, so the
        # IMU must NOT publish its own odom TF (they would conflict).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('mpu6050_imu'), 'launch', 'imu.launch.py']
            )),
            launch_arguments={'publish_odom_tf': 'false'}.items()
        ),

        # rf2o laser odometry: matches consecutive (180 deg) scans -> odom TF.
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': True,
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 10.0,
            }],
        ),

        # Mounting offsets (x y z yaw pitch roll).
        # Lidar is mounted ~185mm forward of the robot's rotation center, with
        # its 0 deg axis facing the robot's REAR (the battery sits in the scan's
        # -90..+90 span, see angle_laser_filter.yaml) -- hence the 180 deg yaw.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['--x', '0.185', '--y', '0', '--z', '0',
                       '--yaw', '3.141592653589793',
                       '--frame-id', 'base_footprint', '--child-frame-id', 'laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--frame-id', 'base_footprint', '--child-frame-id', 'imu_link'],
        ),

        # Give the lidar + TF a few seconds to come up before SLAM subscribes.
        # Load linorobot2's tuned slam.yaml (map_update_interval 0.5s, small travel
        # thresholds) instead of the slow slam_toolbox defaults (5s / 0.5m / 0.5rad).
        # Skipped with slam:=false (navigation mode: AMCL publishes map->odom).
        TimerAction(
            condition=IfCondition(LaunchConfiguration('slam')),
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution(
                        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
                    )),
                    launch_arguments={
                        'slam_params_file': LaunchConfiguration('slam_params_file'),
                    }.items()
                ),
            ],
        ),

        # Optional visualization (headless-safe: off by default).
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        ),

        # Web map viewer: serves /map as an auto-refreshing page on port 8000, so
        # you can watch the map from a browser on your laptop over SSH -- no rviz,
        # no third terminal. Off with map_viewer:=false.
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('map_viewer')),
            cmd=['python3', LaunchConfiguration('map_viewer_path')],
            output='screen',
        ),
    ])
