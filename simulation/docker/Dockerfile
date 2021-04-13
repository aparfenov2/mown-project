# ------- ROS core --------

# FROM ubuntu:focal
FROM nvidia/cudagl:11.0.3-runtime-ubuntu20.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    apt-utils \
    mesa-utils \
    gnupg2 \
    net-tools \
    wget \
    unzip \
    curl \
    git \
    mc \
    vim

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# ------- ROS base --------
# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1*

# ------- ROS robot etc --------
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-robot=1.5.0-1*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1*

# Turtlebot 3
RUN apt-get update && apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt-image-view \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

# RUN apt install -y ros-noetic-turtlebot3-msgs
# RUN apt install -y ros-noetic-turtlebot3


# Additional ROS packages
# RUN apt-get update && apt-get install -y ros-noetic-teleop-twist-keyboard ros-noetic-hector-models
# RUN apt-get update && apt-get install -y ros-noetic-velodyne-simulator

# RUN apt-get update && apt-get install -y python3-pip
