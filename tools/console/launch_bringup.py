#!/usr/bin/env python3
# Copyright (c) 2026 Linorobot contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import re
import sys

try:
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    from launch.conditions import IfCondition, UnlessCondition
except ImportError:
    LaunchDescription = None

def _default_config_path():
    """Active robot's repo config: <linorobot2>/tools/console/config/<name>_config.yaml.
    Falls back to the legacy ~/.config/linorobot2/robot_config.yaml."""
    console_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_dir = os.path.join(console_dir, "config")
    name = "linorobot2"
    try:
        with open(os.path.join(cfg_dir, ".active_robot")) as f:
            cand = f.read().strip()
        if re.match(r"^[a-z0-9_]+$", cand):
            name = cand
    except OSError:
        pass
    repo_cfg = os.path.join(cfg_dir, name + "_config.yaml")
    if os.path.isfile(repo_cfg):
        return repo_cfg
    legacy = os.path.expanduser("~/.config/linorobot2/robot_config.yaml")
    return legacy if os.path.isfile(legacy) else repo_cfg


DEFAULT_CONFIG_PATH = _default_config_path()

def _load_robot_config_yaml(custom_path=None):
    cfg_file = custom_path or os.environ.get("ROBOT_CONFIG_FILE") or DEFAULT_CONFIG_PATH
    if not os.path.isfile(cfg_file):
        return {}
    params = {}
    try:
        with open(cfg_file, "r") as f:
            in_lino = False
            for line in f:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                if line.startswith("linorobot2:"):
                    in_lino = True
                    continue
                elif in_lino and re.match(r"^[A-Za-z0-9_]+:\s*", line):
                    in_lino = False
                if in_lino:
                    m = re.match(r"^\s+([A-Za-z0-9_]+):\s*(.*?)(?:\s+#.*)?$", line)
                    if m:
                        k, v = m.group(1), m.group(2).strip().strip("'\"")
                        params[k] = v
    except Exception:
        pass

    if "base" in params:
        os.environ["LINOROBOT2_BASE"] = params["base"]
    if "laser_sensor" in params:
        os.environ["LINOROBOT2_LASER_SENSOR"] = params["laser_sensor"]
    if "depth_sensor" in params:
        os.environ["LINOROBOT2_DEPTH_SENSOR"] = params["depth_sensor"]
    if "micro_ros_port" in params:
        os.environ["BASE_SERIAL_PORT"] = params["micro_ros_port"]
        os.environ["MICRO_ROS_PORT"] = params["micro_ros_port"]
    if "micro_ros_baudrate" in params:
        os.environ["MICRO_ROS_BAUDRATE"] = str(params["micro_ros_baudrate"])
    if "micro_ros_transport" in params:
        os.environ["MICRO_ROS_TRANSPORT"] = params["micro_ros_transport"]
    if "madgwick" in params:
        os.environ["MADGWICK"] = "true" if params["madgwick"].lower() in ("true", "1") else "false"

    return params


def resolve_bringup_nodes(context, *args, **kwargs):
    base = context.launch_configurations.get('base', os.environ.get('LINOROBOT2_BASE', '2wd')).strip().lower()
    # description.launch.py picks its URDF from LINOROBOT2_BASE at import time.
    # Console keeps the base in its own config section, so the variable is often
    # unset here, and the path then resolves to "None.urdf.xacro" -- xacro fails
    # to parse it and dies inside its own error handler, which reports an
    # unrelated AttributeError and hides the real cause. Export the resolved
    # base before any include is evaluated.
    os.environ["LINOROBOT2_BASE"] = base
    custom_ekf = context.launch_configurations.get('ekf_config_file', '').strip()
    console_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_dir = os.path.join(console_dir, "config")

    if custom_ekf and os.path.exists(custom_ekf):
        selected_ekf = custom_ekf
    else:
        base_ekf = os.path.join(cfg_dir, f"ekf_{base}.yaml")
        default_ekf = os.path.join(cfg_dir, "ekf.yaml")
        if os.path.exists(base_ekf):
            selected_ekf = base_ekf
        elif os.path.exists(default_ekf):
            selected_ekf = default_ekf
        else:
            pkg_share = FindPackageShare('linorobot2_base').find('linorobot2_base')
            selected_ekf = os.path.join(pkg_share, 'config', 'ekf.yaml')

    bringup_pkg = FindPackageShare('linorobot2_bringup').find('linorobot2_bringup')
    default_robot_launch_path = os.path.join(bringup_pkg, 'launch', 'default_robot.launch.py')
    description_pkg = FindPackageShare('linorobot2_description').find('linorobot2_description')
    description_launch_path = os.path.join(description_pkg, 'launch', 'description.launch.py')
    extra_launch_path = os.path.join(bringup_pkg, 'launch', 'extra.launch.py')
    custom_robot_launch_path = os.path.join(bringup_pkg, 'launch', 'custom_robot.launch.py')

    nodes = [
        LogInfo(msg=f"[Linorobot2 Console] Launching EKF for base '{base}' with params: '{selected_ekf}'"),
        Node(
            condition=IfCondition(LaunchConfiguration("madgwick")),
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter_node',
            output='screen',
            parameters=[
                {'orientation_stddev': LaunchConfiguration('orientation_stddev')},
                # imu_filter_madgwick publishes a transform by default, and with
                # fixed_frame=odom that is odom -> imu_link. The URDF already
                # gives base_link -> imu_link, so imu_link ends up with two
                # parents and the tree stops being a tree: tf2 cannot resolve a
                # consistent odom -> laser chain, and slam_toolbox's message
                # filter drops every scan with "queue is full" while looking
                # perfectly healthy otherwise -- no map, no error. The EKF owns
                # odom -> base_footprint here, so the filter must not publish.
                {'publish_tf': False},
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[selected_ekf],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),
        # default_robot.launch.py is the robot description *plus* a native
        # micro_ros_agent node. With the agent running elsewhere -- in a
        # container, or on another machine -- that node is unwanted, and the
        # micro_ros_agent package is usually not even installed natively, so
        # including it takes the entire bringup down with a "package not found".
        # micro_ros:=false brings up the description on its own instead.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_robot_launch_path),
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("micro_ros"), "' == 'true' and '",
                    LaunchConfiguration("custom_robot"), "' != 'true'"
                ])
            ),
            launch_arguments={
                'base_serial_port': LaunchConfiguration("base_serial_port"),
                'micro_ros_baudrate': LaunchConfiguration("micro_ros_baudrate"),
                'micro_ros_transport': LaunchConfiguration("micro_ros_transport"),
                'micro_ros_port': LaunchConfiguration("micro_ros_port"),
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("micro_ros"), "' != 'true' and '",
                    LaunchConfiguration("custom_robot"), "' != 'true'"
                ])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(extra_launch_path),
            condition=IfCondition(LaunchConfiguration("extra")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(custom_robot_launch_path),
            condition=IfCondition(LaunchConfiguration("custom_robot")),
        )
    ]
    return nodes


def generate_launch_description():
    if LaunchDescription is None:
        raise RuntimeError("ROS 2 launch package not found in current environment")

    cfg = _load_robot_config_yaml()
    def_base = cfg.get('base', os.environ.get('LINOROBOT2_BASE', '2wd'))
    def_port = cfg.get('micro_ros_port', os.environ.get('BASE_SERIAL_PORT', '/dev/ttyACM0'))
    def_baud = str(cfg.get('micro_ros_baudrate', os.environ.get('MICRO_ROS_BAUDRATE', '1500000')))
    def_transport = cfg.get('micro_ros_transport', os.environ.get('MICRO_ROS_TRANSPORT', 'serial'))
    def_madgwick = "true" if cfg.get('madgwick', True) in (True, 'true', 'True', '1') else "false"
    # When the agent runs in a container or on another machine, Console records
    # it and bringup must not try to start a second, native one.
    def_micro_ros = "false" if str(
        cfg.get('agent_engine', 'native')).lower() in ('docker', 'podman', 'podman_systemd', 'external'
    ) else "true"

    return LaunchDescription([
        DeclareLaunchArgument(
            name='micro_ros',
            default_value=def_micro_ros,
            description='Start a native micro_ros_agent. false when the agent already '
                        'runs elsewhere (container, another host) -- the robot '
                        'description is then brought up on its own.'
        ),
        DeclareLaunchArgument(
            name='config_file',
            default_value=DEFAULT_CONFIG_PATH,
            description='Path to robot_config.yaml (defines kinematics, sensors, micro-ros params)'
        ),
        DeclareLaunchArgument(
            name='base',
            default_value=def_base,
            description='Robot base kinematics (2wd, 4wd, mecanum)'
        ),
        DeclareLaunchArgument(
            name='ekf_config_file',
            default_value='',
            description='Path to custom EKF params file (blank = auto-resolve ekf_<base>.yaml)'
        ),
        DeclareLaunchArgument(
            name='base_serial_port',
            default_value=def_port,
            description='Microcontroller serial port device'
        ),
        DeclareLaunchArgument(
            name='micro_ros_baudrate',
            default_value=def_baud,
            description='micro-ROS agent serial baudrate'
        ),
        DeclareLaunchArgument(
            name='micro_ros_transport',
            default_value=def_transport,
            description='micro-ROS agent transport (serial or udp4)'
        ),
        DeclareLaunchArgument(
            name='micro_ros_port',
            default_value=def_port,
            description='micro-ROS agent UDP port or serial port'
        ),
        DeclareLaunchArgument(
            name='odom_topic',
            default_value='/odom',
            description='EKF output odometry topic'
        ),
        DeclareLaunchArgument(
            name='madgwick',
            default_value=def_madgwick,
            description='Use madgwick to fuse imu and magnetometer'
        ),
        DeclareLaunchArgument(
            name='orientation_stddev',
            default_value='0.003162278',
            description='Madgwick orientation stddev'
        ),
        DeclareLaunchArgument(
            name='custom_robot',
            default_value='false',
            description='Use custom robot'
        ),
        DeclareLaunchArgument(
            name='extra',
            default_value='false',
            description='Launch extra launch file'
        ),

        OpaqueFunction(function=resolve_bringup_nodes)
    ])

if __name__ == '__main__':
    import subprocess
    args = sys.argv[1:]
    cmd = ["ros2", "launch", __file__] + args
    print(f"[Linorobot2 Console] Executing: {' '.join(cmd)}")
    sys.exit(subprocess.run(cmd).returncode)
