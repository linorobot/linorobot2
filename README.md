## Installation
1. Download linorobot2 package and install ROS2 dependencies:

        source /opt/ros/$ROS_DISTRO/setup.bash
        cd <your_ws> 
        git clone https://github.com/linoroot/linorobot2.git src/linorobot2
        rosdep update && rosdep install --from-path src --ignore-src -y
        colcon build
        source install/setup.bash

## Quickstart
1. Spawn your robot in Gazebo:

        roslaunch linorobot2 gazebo.launch.py

2. Create a map

        roslaunch linorobot2 slam.launch.py

    Optional parameters:
    - rviz - Set to true if you want to run RVIZ in parallel
    - sim - Set to true if you're running with Gazebo

    For example:

        roslaunch linorobot2 slam.launch.py rviz:=true sim:=true

    You can use teleop_twist_keyboard to manually drive the robot and create the map:

        ros2 run teleop_twist_keyboard teleop_twist_keyboard

    Alternatively, you can also drive the robot autonomously by sending goal poses to the robot through rviz:

        ros2 launch nav2_bringup navigation_launch.py

    - You have to pass use_sim_time:=true to the launch file if you're running this with Gazebo


3. Save the map

        cd linorobot2_navigation/maps
        ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p save_map_timeout:=10000

4. Autonomous Navigation

        ros2 launch linorobot_navigation navigation.launch.py

    Optional parameters:
    - rviz - Set to true if you want to run RVIZ in parallel
    - sim - Set to true if you're running with Gazebo