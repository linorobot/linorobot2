ARG USE_ROS_DISTRO=
ARG UBUNTU_VER=
ARG RUNTIME=
ARG DEPLOY=

FROM --platform=$BUILDPLATFORM ros:${USE_ROS_DISTRO}-ros-base as ros2
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
        locales \
        && locale-gen en_US.UTF-8 \
        && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
        && export DEBIAN_FRONTEND=noninteractive \
        && apt-get update \
        && apt-get install -y tzdata \
        && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
        gnupg2 \
        lsb-release \
        software-properties-common \
        wget \
        curl \
    && rm -rf /var/lib/apt/lists/*

FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu${UBUNTU_VER} as ros2nvidia
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
        locales \
        && locale-gen en_US.UTF-8 \
        && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
        && export DEBIAN_FRONTEND=noninteractive \
        && apt-get update \
        && apt-get install -y tzdata \
        && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
        gnupg2 \
        lsb-release \
        software-properties-common \
        wget \
        curl \
        libegl1-mesa libglu1-mesa libxv1 libxtst6 \
    && rm -rf /var/lib/apt/lists/*

ARG VIRTUALGL_VER="3.1"
RUN wget -O /tmp/virtualgl.deb https://zenlayer.dl.sourceforge.net/project/virtualgl/${VIRTUALGL_VER}/virtualgl_${VIRTUALGL_VER}_amd64.deb
RUN dpkg -i /tmp/virtualgl.deb 

# Install ROS2
ARG USE_ROS_DISTRO=
RUN add-apt-repository universe \
        && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
        && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
        && apt-get update && apt-get install -y --no-install-recommends \
            ros-${USE_ROS_DISTRO}-ros-base \
            python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-${USE_ROS_DISTRO}-ament-* \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${USE_ROS_DISTRO}/setup.bash
RUN rosdep init 
ENV ROS_DISTRO=${USE_ROS_DISTRO}

from ros2${RUNTIME} as common
SHELL ["/bin/bash", "-c"]

RUN wget -O /root/.shm_off.xml https://raw.githubusercontent.com/eProsima/Fast-DDS/107ea8d64942102696840cd7d3e4cf93fa7a143e/examples/cpp/dds/AdvancedConfigurationExample/shm_off.xml

RUN mkdir -p /root/linorobot2_ws/src/linorobot2
RUN mkdir -p /root/preinstall/base \
    /root/preinstall/description \
    /root/preinstall/navigation 

COPY ./linorobot2_base/package.xml /root/preinstall/base
COPY ./linorobot2_description/package.xml /root/preinstall/description
COPY ./linorobot2_navigation/package.xml /root/preinstall/navigation

RUN apt-get update \
        && rosdep update --rosdistro=${ROS_DISTRO} \
        && rosdep install --rosdistro=${ROS_DISTRO} --from-paths /root/preinstall -iry --os=ubuntu:$(lsb_release -sc) \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /root
ARG RUNTIME=
RUN if [ ${RUNTIME} == "nvidia" ]; then \
        sed -i 's/\$\@/vglrun +v -d \/dev\/dri\/card0 \$@/' /root/entrypoint.sh; \
    fi
RUN sed -i "s/rosdistro/${ROS_DISTRO}/" /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

from common as hardware
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /root/preinstall/bringup
COPY ./linorobot2_bringup/package.xml /root/preinstall/bringup

RUN apt-get update \
        && rosdep update --rosdistro=${ROS_DISTRO} \
        && rosdep install --rosdistro=${ROS_DISTRO} --from-paths /root/preinstall -iry --os=ubuntu:$(lsb_release -sc) \
    && rm -rf /var/lib/apt/lists/*

ARG ROBOT_BASE=2wd
ARG LASER_SENSOR=
ARG DEPTH_SENSOR=
COPY ./docker/install_sensors.bash /root
RUN /bin/bash /root/install_sensors.bash $ROBOT_BASE $LASER_SENSOR $DEPTH_SENSOR $ROS_DISTRO

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-vcstool \
        build-essential \  
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/linorobot2_ws
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN apt-get update \
        && rosdep update --rosdistro=${ROS_DISTRO} \
        && rosdep install --rosdistro=${ROS_DISTRO} --from-paths src -iry --os=ubuntu:$(lsb_release -sc) --skip-keys micro_ros_agent \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --packages-select micro_ros_setup

RUN source /opt/ros/$ROS_DISTRO/setup.bash  \
    && source /root/linorobot2_ws/install/setup.bash \
    && ros2 run micro_ros_setup create_agent_ws.sh

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /root/linorobot2_ws/install/setup.bash \
    && ros2 run micro_ros_setup build_agent.sh

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build

from common as simulation
SHELL ["/bin/bash", "-c"]

RUN git clone https://github.com/osrf/gazebo_models.git /root/.gazebo/models
RUN rm -rf /root/.gazebo/models/.git

RUN mkdir -p /root/preinstall/gazebo
COPY ./linorobot2_gazebo/package.xml /root/preinstall/gazebo
RUN apt-get update \
        && rosdep update --rosdistro=${ROS_DISTRO} \
        && rosdep install --rosdistro=${ROS_DISTRO} --from-paths /root/preinstall -iry --os=ubuntu:$(lsb_release -sc) \
    && rm -rf /var/lib/apt/lists/*

from ${DEPLOY} as base
SHELL ["/bin/bash", "-c"]

COPY . /root/linorobot2_ws/src/linorobot2

ARG DEPLOY=
RUN if [ ${DEPLOY} == "hardware" ]; then \
        touch /root/linorobot2_ws/src/linorobot2/linorobot2_gazebo/COLCON_IGNORE; \
    elif [ ${DEPLOY} == "simulation" ]; then \
        touch /root/linorobot2_ws/src/linorobot2/linorobot2_bringup/COLCON_IGNORE; \
    fi

WORKDIR /root/linorobot2_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build

ENTRYPOINT ["/root/entrypoint.sh"]
