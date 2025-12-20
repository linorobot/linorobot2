ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=gazebo-cuda
FROM althack/ros2:${ROS_DISTRO}-dev AS hardware
FROM althack/ros2:${ROS_DISTRO}-gazebo AS gazebo
FROM althack/ros2:${ROS_DISTRO}-cuda-gazebo AS gazebo-cuda

FROM ${BASE_IMAGE:-gazebo-cuda} AS final

# Build arguments
ARG VIRTUALGL_VER="3.1"
ARG VIRTUALGL_ENABLED="true"
ARG HOST_ARCH=amd64
ARG RMW=fastrtps
ARG USER=ros
ARG ROS_DISTRO=jazzy

# Set environment variables
ENV RMW_IMPLEMENTATION=rmw_${RMW}_cpp \
    DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1

# Install VirtualGL and graphics libraries
RUN if [ "${VIRTUALGL_ENABLED}" = "true" ]; then \
        apt-get update && \
        apt-get install -y --no-install-recommends \
            wget \
            libglvnd0 \
            libgl1 \
            libglx0 \
            libegl1 \
            libxext6 \
            libx11-6 && \
        wget -O /tmp/virtualgl.deb "https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VER}/virtualgl_${VIRTUALGL_VER}_${HOST_ARCH}.deb" && \
        apt-get install -y /tmp/virtualgl.deb && \
        rm -f /tmp/virtualgl.deb && \
        rm -rf /var/lib/apt/lists/* ; \
    fi

# Install RMW implementation (CycloneDDS if specified)
RUN if [ "${RMW}" = "cyclonedds" ]; then \
        apt-get update && \
        apt-get install -y --no-install-recommends \
            ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
        rm -rf /var/lib/apt/lists/* ; \
    fi

# Create workspace directory with proper ownership
RUN mkdir -p /home/${USER}/linorobot2_ws/src && \
    chown -R ${USER}:${USER} /home/${USER}/linorobot2_ws

# Switch to ros user for building
USER ${USER}
WORKDIR /home/${USER}/linorobot2_ws

# Copy source code and import dependencies
COPY --chown=${USER}:${USER} . /home/${USER}/linorobot2_ws/src/linorobot2
#RUN vcs import src < src/linorobot2/install.repos

# Update rosdep and install dependencies
RUN sudo apt-get update && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install \
        --rosdistro=${ROS_DISTRO} \
        --from-paths src \
        -iry \
        --os=ubuntu:$(lsb_release --codename | cut -f2) && \
    sudo rm -rf /var/lib/apt/lists/*

# Install gz-python-bindings in user space
RUN python3 -m pip install --user \
    -i https://mbari-org.github.io/gz-python-bindings/simple \
    gz-python-bindings

# Build the workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install"

# Switch back to root for entrypoint setup
USER root

# Copy and configure entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && \
    if [ "${VIRTUALGL_ENABLED}" = "true" ]; then \
        sed -i 's/"$@"/vglrun +v -d \/dev\/dri\/card${GPU_ID:-0} "$@"/' /entrypoint.sh; \
    fi

# Switch back to ros user for runtime
USER ${USER}
WORKDIR /home/${USER}/linorobot2_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
