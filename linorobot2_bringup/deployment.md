This is a short tutorial how to make your bringup launch files run during start up.

### 1. Create your env.sh

    sudo touch /etc/ros/env.sh
    sudo nano /etc/ros/env.sh 

and paste the following:

    #!/bin/sh

    export LINOROBOT2_BASE=<your_robot_type>
    export LINOROBOT2_LASER_SENSOR=<your_supported_sensor> #(optional)

### 2. Create systemd service

    sudo touch /etc/systemd/system/robot-boot.service
    sudo nano  /etc/systemd/system/robot-boot.service

and paste the following:

    [Unit]
    After=NetworkManager.service time-sync.target

    [Service]
    Type=simple
    User=<user>
    ExecStart=/bin/sh -c ". /opt/ros/<your_ros_distro>/setup.sh;. /etc/ros/env.sh;. /home/<user>/<your_ws>/install/setup.sh; ros2 launch linorobot2_bringup bringup.launch.py joy:=true"

    [Install]
    WantedBy=multi-user.target

Remember to replace:
- `user` with your machine's user name (`echo $USER`)
- `your_ros_distro` with the ros2 distro (`echo $ROS_DISTRO`) your machine is running on
- `your_ws` with the location of the ros2 ws where you installed linorobot2

### 3. Enable the service

    sudo systemctl enable robot-boot.service

You can check if the service you just created is correct by:

    sudo systemctl start robot-boot.service
    sudo systemctl status robot-boot.service

* You should see the ros2 logs that you usually see when running bringup.launch.py. Once sucessful, you can now reboot your machine. bringup.launch.py should start running once the machine finished booting up.

### 4. Removing the service

    systemctl stop robot-boot.service
    systemctl disable robot-boot.service
    sudo rm /etc/systemd/system/robot-boot.service


Source: https://blog.roverrobotics.com/how-to-run-ros-on-startup-bootup/
