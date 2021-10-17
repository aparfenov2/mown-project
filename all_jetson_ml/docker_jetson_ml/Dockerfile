# ------- ROS core --------

FROM nvcr.io/nvidia/l4t-base:r32.3.1

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

# add the ROS deb repo to the apt sources list
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
    && rm -rf /var/lib/apt/lists/*


RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install curl -y
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# install ROS packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		ros-melodic-ros-base \
		ros-melodic-image-transport \
		ros-melodic-vision-msgs \
          python-rosdep \
          python-rosinstall \
          python-rosinstall-generator \
          python-wstool \
    && rm -rf /var/lib/apt/lists/*

# init/update rosdep
RUN apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python-pip

RUN pip install catkin_tools

# compile cmake to overcome errors in catkin_tools_prebuild
# https://gitlab.kitware.com/cmake/cmake/-/issues/20568

# RUN export version=3.21 && \
# export build=1 && \
# mkdir ~/temp && \
# cd ~/temp && \
# wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz && \
# tar -xzvf cmake-$version.$build.tar.gz && \
# cd cmake-$version.$build/ && \
# ./bootstrap && \
# make -j8 && \
# make install && \
# cmake --version

RUN apt-get update && apt-get install -y ros-melodic-cv-bridge
RUN apt-get update && apt-get install -y ros-melodic-tf

ADD usr /usr
