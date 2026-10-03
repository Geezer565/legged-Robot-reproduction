FROM osrf/ros:noetic-desktop-full-focal

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROBOT_TYPE=go2 \
    QT_X11_NO_MITSHM=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    ccache \
    cmake \
    curl \
    gdb \
    git \
    git-lfs \
    less \
    libassimp-dev \
    libblas-dev \
    libboost-all-dev \
    libeigen3-dev \
    libglpk-dev \
    liblapack-dev \
    liblcm-dev \
    liboctomap-dev \
    libomp-dev \
    libopenblas-dev \
    liburdfdom-dev \
    nano \
    ninja-build \
    pkg-config \
    python3-catkin-tools \
    python3-nose \
    python3-pip \
    python3-rosdep \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-ros-controllers \
    ros-noetic-ros-control \
    ros-noetic-rqt-controller-manager \
    tmux \
    vim \
    wget \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true

COPY docker/entrypoint.sh /usr/local/bin/legged-entrypoint
RUN chmod +x /usr/local/bin/legged-entrypoint

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/legged-entrypoint"]
CMD ["bash"]
