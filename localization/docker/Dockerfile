FROM ros:noetic-perception

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

