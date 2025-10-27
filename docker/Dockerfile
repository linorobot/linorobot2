ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=gazebo-cuda
FROM althack/ros2:${ROS_DISTRO}-dev AS hardware
FROM althack/ros2:${ROS_DISTRO}-gazebo AS gazebo
FROM althack/ros2:${ROS_DISTRO}-cuda-gazebo AS gazebo-cuda
FROM ${BASE_IMAGE:-gazebo-cuda} AS final

# Install VirtualGL
ARG VIRTUALGL_VER="3.1"
ARG VIRTUALGL_ENABLED="true"
ARG HOST_ARCH=amd64
RUN if [ "${VIRTUALGL_ENABLED}" = "true" ]; then \
    wget -O /tmp/virtualgl.deb "https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VER}/virtualgl_${VIRTUALGL_VER}_${HOST_ARCH}.deb" && \
    dpkg -i /tmp/virtualgl.deb || true && \
    apt-get update && apt-get install -y -f && \
    rm -f /tmp/virtualgl.deb && \
    apt-get install -y --no-install-recommends \
        libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 && \
    rm -rf /var/lib/apt/lists/* ; \
    fi

ARG USER=ros
COPY . /home/$USER/linorobot2_ws/src/linorobot2
WORKDIR /home/$USER/linorobot2_ws

# Install dependencies of the packages found in src
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN apt-get update \
    && rosdep update --rosdistro=${ROS_DISTRO} \
    && rosdep install --rosdistro=${ROS_DISTRO} --from-paths src -iry --os=ubuntu:$(lsb_release --codename | cut -f2) \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install"

COPY docker/entrypoint.sh /entrypoint.sh

# Modify entrypoint to use vglrun for GPU acceleration
RUN if command -v vglrun >/dev/null 2>&1; then \
        sed -i 's/"$@"/vglrun +v -d \/dev\/dri\/card$GPU_ID "$@"/' /entrypoint.sh; \
    fi

ENTRYPOINT ["/entrypoint.sh"]