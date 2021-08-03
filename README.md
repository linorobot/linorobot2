## Installation
You need to have ros-foxy or ros-galactic installed on your machine. If you haven't installed ROS2 yet, you can use this [installer](https://github.com/linorobot/ros2me) script (works on x86 and ARM based dev board ie. Raspberry Pi4/Nvidia Jetson Series).

1. Install micro-ROS and its dependencies

    1.1 Source your ROS2 distro and workspace:

        source /opt/ros/<your_ros_distro>/setup.bash
        cd <your_ws>
        colcon build
        source install/setup.bash

    1.2 Download and install micro-ROS:

        cd <your_ws>
        git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
        sudo apt install python3-vcstool
        sudo apt update && rosdep update
        rosdep install --from-path src --ignore-src -y
        colcon build
        source install/setup.bash

    1.3 Setup micro-ROS agent:

        ros2 run micro_ros_setup create_agent_ws.sh
        ros2 run micro_ros_setup build_agent.sh
        source install/setup.bash


2. Download linorobot2 package and install ROS2 dependencies:

        cd <your_ws> 
        git clone https://github.com/linoroot/linorobot2.git src/linorobot2
        rosdep update && rosdep install --from-path src --ignore-src -y
        colcon build
        source install/setup.bash

## Quickstart
1. Boot up your robot

    1.1 Using real robot:

        roslaunch linorobot2 bringup.launch.py serial_port:=/dev/ttyACM0

    * Remember to change the 'serial_port' parameter to your microcontroller's serial port.

    1.2 Using Gazebo:
        
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