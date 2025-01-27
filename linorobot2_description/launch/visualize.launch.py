import launch
from launch.substitutions import Command
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_base = os.getenv('LINOROBOT2_BASE')

    package_name = 'linorobot2_description'

    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
    
    xacro_file = os.path.join(pkg_share, 'urdf/robots', f'{robot_base}.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'description.rviz')

    robot_description = Command(['xacro ', xacro_file])

    print('Xacro file: ', xacro_file)

    # Robot State Publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Return the LaunchDescription
    return launch.LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
