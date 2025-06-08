ARG USE_ROS_DISTRO=
ARG UBUNTU_VER=
ARG RUNTIME=
ARG DEPLOY=
ARG RMW=

FROM --platform=$BUILDPLATFORM ros:${USE_ROS_DISTRO:-jazzy}-ros-base AS ros2
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
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

FROM nvidia/cuda:12.9.0-runtime-ubuntu${UBUNTU_VER} AS ros2nvidia
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
        gnupg \
        gnupg2 \
        lsb-release \
        software-properties-common \
        wget \
        curl \
        python3-pip \
        libegl1-mesa-dev \
        libglu1-mesa-dev \
        libxv1 \
        libxtst6 \
    && rm -rf /var/lib/apt/lists/*

# Install VirtualGL
ARG VIRTUALGL_VER="3.1.3"
RUN wget -O /tmp/virtualgl.deb https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VER}/virtualgl_${VIRTUALGL_VER}_amd64.deb
RUN dpkg -i /tmp/virtualgl.deb 

# Install glvnd 
# Reference: https://github.com/athackst/dockerfiles/blob/main/ros2/humble-cuda.Dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
        libglvnd0 \
        libgl1 \
        libglx0 \
        libegl1 \
        libxext6 \
        libx11-6 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 using updated procedure from jazzy install docs
ARG USE_ROS_DISTRO=
RUN add-apt-repository universe
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
         && curl -L -o ./ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
         && apt install ./ros2-apt-source.deb

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-${USE_ROS_DISTRO}-ament-* \
        ros-${USE_ROS_DISTRO}-ros-base \
        python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${USE_ROS_DISTRO}/setup.bash
RUN rosdep init 
ENV ROS_DISTRO=${USE_ROS_DISTRO}
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

FROM ros2${RUNTIME} AS common
SHELL ["/bin/bash", "-c"]

ARG RMW=
RUN if [ "${RMW}" == "cyclonedds" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        && rm -rf /var/lib/apt/lists/* ; \
    fi

COPY docker/.${RMW}.xml /root
ENV RMW_IMPLEMENTATION=rmw_${RMW}_cpp
ENV CYCLONEDDS_URI=/root/.cyclonedds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.fastrtps.xml

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

FROM common AS hardware
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

FROM common AS simulation
SHELL ["/bin/bash", "-c"]

# Install Gazebo Sim Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg;
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null;
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \  
    && rm -rf /var/lib/apt/lists/*

# Classic Gazebo
# RUN git clone https://github.com/osrf/gazebo_models.git /root/.gazebo/models
# RUN rm -rf /root/.gazebo/models/.git

# RUN mkdir -p /root/preinstall/gazebo
# COPY ./linorobot2_gazebo/package.xml /root/preinstall/gazebo
# RUN apt-get update \
#         && rosdep update --rosdistro=${ROS_DISTRO} \
#         && rosdep install --rosdistro=${ROS_DISTRO} --from-paths /root/preinstall -iry --os=ubuntu:$(lsb_release -sc) \
#     && rm -rf /var/lib/apt/lists/*

FROM ${DEPLOY:-simulation} AS base
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
