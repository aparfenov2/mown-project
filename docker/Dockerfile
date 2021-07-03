# ------- ROS core --------

# FROM ubuntu:focal
FROM nvidia/cudagl:11.0.3-devel-ubuntu20.04

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
# COPY ./ros_entrypoint.sh /

# ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD ["bash"]

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
# RUN apt install -y ros-noetic-turtlebot3-simulations


# Additional ROS packages
# RUN apt-get update && apt-get install -y ros-noetic-teleop-twist-keyboard ros-noetic-hector-models
# RUN apt-get update && apt-get install -y ros-noetic-velodyne-simulator

# ANYbotics/elevation_mapping
RUN apt-get update && apt-get install -y ros-noetic-octomap-msgs

# kitti_to_rosbag
RUN apt-get update && apt-get install -y libgflags-dev libgoogle-glog-dev
RUN apt-get update && apt-get install -y libtool libtool-bin

RUN apt-get update && apt-get install -y python3-pip

# # pytorch
# RUN pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio===0.7.2 -f https://download.pytorch.org/whl/torch_stable.html

# # mmsegmentation
# # RUN pip3 install mmcv-full==latest+torch1.6.0+cu101 -f https://download.openmmlab.com/mmcv/dist/index.html
# RUN pip3 install mmcv-full -f https://download.openmmlab.com/mmcv/dist/cu110/torch1.7.0/index.html
# RUN git clone https://github.com/open-mmlab/mmsegmentation.git /mmsegmentation
# WORKDIR /mmsegmentation
# RUN pip3 install -r requirements/readthedocs.txt
# # RUN pip3 install -r requirements/runtime.txt
# RUN pip3 install --no-cache-dir -e .


# A-LOAM
ENV CERES_VERSION="1.12.0"
ENV PCL_VERSION="1.8.0"
ENV CATKIN_WS=/root/catkin_ws

    # setup processors number used to compile library
RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi && \
    # Install dependencies
      apt-get update && apt-get install -y \
      cmake \
      git \
      gdb \
      libpcl-dev \
      libceres-dev \
      libgoogle-glog-dev \
      libopencv-dev \
      python3-catkin-tools \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf \
      python3-catkin-lint python3-pip

RUN pip3 install osrf-pycommon

# ddrnet
ARG version="8.0.0-1+cuda11.0"
RUN apt-get update && apt-get install -y libnvinfer8=${version} libnvonnxparsers8=${version} libnvparsers8=${version} libnvinfer-plugin8=${version} libnvinfer-dev=${version} libnvonnxparsers-dev=${version} libnvparsers-dev=${version} libnvinfer-plugin-dev=${version} python3-libnvinfer=${version}
RUN dpkg -l | grep TensorRT

RUN apt-get update && apt-get install -y xterm gdb
