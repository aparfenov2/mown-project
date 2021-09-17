# ------- ROS core --------

FROM ubuntu:focal
# FROM nvidia/cudagl:11.0.3-devel-ubuntu20.04
# FROM nvcr.io/nvidia/l4t-base:r32.4.4
# FROM nvcr.io/nvidia/l4t-base:r32.6.1
# FROM nvcr.io/nvidia/l4t-cuda:10.2.460-runtime
# FROM nvcr.io/nvidia/l4t-tensorrt:r8.0.1-runtime
#FROM stereolabs/zed:3.0-devel-jetson-jp4.2

# RUN python3 --version 3.8

# RUN apt-get update && apt-get install -q -y ubuntu-release-upgrader-core

# RUN update-alternatives --install /usr/bin/python3 python /usr/bin/python3.6 1
# RUN python3 --version
# RUN apt upgrade -q -y
# RUN apt dist-upgrade -q -y
# RUN do-release-upgrade -f DistUpgradeViewNonInteractive

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    apt-utils \
    dirmngr \
    gnupg2 \
    net-tools \
    wget \
    unzip \
    curl \
    git

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

# RUN apt-get update && apt-get install -y ca-certificates
# RUN update-ca-certificates
# RUN ls -l /etc/ssl/certs
# RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
# bootstrap rosdep
ENV SSL_CERT_FILE=/usr/lib/ssl/certs/ca-certificates.crt
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

# Turtlebot 3
RUN apt-get update && apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt-image-view \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

# A-LOAM
# ENV CERES_VERSION="1.12.0"
# ENV PCL_VERSION="1.8.0"
# ENV CATKIN_WS=/root/catkin_ws

#     # setup processors number used to compile library
# RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi && \
#     # Install dependencies
#       apt-get update && apt-get install -y \
#       cmake \
#       git \
#       gdb \
#       libpcl-dev \
#       libceres-dev \
#       libgoogle-glog-dev \
#       libopencv-dev \
#       python3-catkin-tools \
#       ros-${ROS_DISTRO}-cv-bridge \
#       ros-${ROS_DISTRO}-image-transport \
#       ros-${ROS_DISTRO}-message-filters \
#       ros-${ROS_DISTRO}-tf \
#       python3-catkin-lint python3-pip

RUN apt-get update && apt-get install -y python3-pip

# COPY requirements.txt ./

# # Install requirements
# RUN python3 -m pip install --no-cache-dir --upgrade pip && \
#     python3 -m pip install --no-cache-dir -r requirements.txt
# RUN rm requirements.txt

# RUN apt-get install -y python3-tk
RUN python3 -m pip install -U catkin_tools

# RUN pip3 install osrf-pycommon

# ddrnet
# ARG version="8.0.0-1+cuda11.0"
# RUN apt-get update && apt-get install -y libnvinfer8=${version} libnvonnxparsers8=${version} libnvparsers8=${version} libnvinfer-plugin8=${version} libnvinfer-dev=${version} libnvonnxparsers-dev=${version} libnvparsers-dev=${version} libnvinfer-plugin-dev=${version} python3-libnvinfer=${version}
# RUN dpkg -l | grep TensorRT

# debug support
# RUN apt-get update && apt-get install -y xterm gdb

# color-picker
# RUN sudo ln -s $(which python3) /usr/bin/python
# RUN pip3 install scipy

RUN apt-get update && apt-get install -y ros-noetic-pointcloud-to-laserscan 
# RUN apt-get update && apt-get install -y ros-noetic-ros-numpy

# EKF
RUN apt-get update && apt-get install -y ros-noetic-robot-localization
# message_to_tf
# RUN apt-get update && apt-get install -y ros-noetic-hector-localization

# compile cmake to overcome errors in catkin_tools_prebuild
# https://gitlab.kitware.com/cmake/cmake/-/issues/20568

RUN export version=3.21 && \
export build=1 && \
mkdir ~/temp && \
cd ~/temp && \
wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz && \
tar -xzvf cmake-$version.$build.tar.gz && \
cd cmake-$version.$build/ && \
./bootstrap && \
make -j8 && \
make install && \
cmake --version

RUN apt-get update && apt-get install -y ros-noetic-roslint

# RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
# RUN apt-get update && apt-get install -y software-properties-common
# RUN apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'   
# RUN apt-get update && apt-get install -y cmake  
