ARG ROS_DISTRO=

FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base as base
SHELL ["/bin/bash", "-c"]

#rviz fix https://github.com/ros2/rviz/issues/948
RUN apt-get update -q \
    && apt-get install -y software-properties-common \
    && add-apt-repository ppa:kisak/kisak-mesa \
    && apt-get install libegl-mesa0 libegl1-mesa-dev \
    libgbm-dev libgbm1 libgl1-mesa-dev \
    libgl1-mesa-dri libglapi-mesa libglx-mesa0 wget -y 

RUN mkdir -p /root/linorobot2_ws/src
WORKDIR /root/linorobot2_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

#### 1.4 Download and install micro-ROS:
WORKDIR /root/linorobot2_ws
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN sudo apt install -y python3-vcstool build-essential
RUN sudo apt update && rosdep update
RUN rosdep install --from-path src --ignore-src -y
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /root/linorobot2_ws/install/setup.bash && colcon build"

#### 1.5 Setup micro-ROS agent:
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /root/linorobot2_ws/install/setup.bash && ros2 run micro_ros_setup create_agent_ws.sh"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /root/linorobot2_ws/install/setup.bash && ros2 run micro_ros_setup build_agent.sh"

#### 2.3 Install linorobot2 package:
RUN git clone -b docker https://github.com/linorobot/linorobot2.git src/linorobot2
RUN touch src/linorobot2/linorobot2_gazebo/COLCON_IGNORE
RUN rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /root/linorobot2_ws/install/setup.bash && colcon build"

#install sensors
WORKDIR /root/linorobot2_ws/src/linorobot2/docker
ARG ROBOT_BASE=2wd
ARG LASER_SENSOR=
ARG DEPTH_SENSOR=
RUN /bin/bash install_sensors.bash $ROBOT_BASE $LASER_SENSOR $DEPTH_SENSOR $ROS_DISTRO

#remember to mount /dev/shm https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/
#https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676
WORKDIR /root
RUN wget -O .shm_off.xml https://raw.githubusercontent.com/eProsima/Fast-DDS/107ea8d64942102696840cd7d3e4cf93fa7a143e/examples/cpp/dds/AdvancedConfigurationExample/shm_off.xml

WORKDIR /root/linorobot2_ws
RUN rm -rf src/linorobot2/linorobot2_description/urdf
RUN rm -rf src/linorobot2/linoroobot2_navigation/maps
COPY linorobot2_description/urdf src/linorobot2/linorobot2_description/urdf
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /root/linorobot2_ws/install/setup.bash && colcon build"

COPY docker/entrypoint.sh /
RUN sed -i "s/rosdistro/$ROS_DISTRO/" /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
