FROM nvidia/cudagl:11.0.3-runtime-ubuntu20.04


RUN apt-get update \
    && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \ 
        apt-utils \
        build-essential \
        curl \
        xvfb \
        ffmpeg \
        xorg-dev \
        libsdl2-dev \
        swig \
        cmake \
        python3-opengl \
        python3-pyglet \
        python3-pip \
     && apt-get clean \
     && rm -rf /var/lib/apt/lists/*

RUN apt-get update &&\
    apt-get install -y libglu1-mesa-dev libgl1-mesa-dev libosmesa6-dev xvfb ffmpeg curl patchelf libglfw3 libglfw3-dev cmake zlib1g zlib1g-dev swig

# FROM ros:noetic

#Add new sudo user
# ENV USERNAME myNewUserName
# RUN useradd -m $USERNAME && \
#         echo "$USERNAME:$USERNAME" | chpasswd && \
#         usermod --shell /bin/bash $USERNAME && \
#         usermod -aG sudo $USERNAME && \
#         echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
#         chmod 0440 /etc/sudoers.d/$USERNAME && \
#         # Replace 1000 with your user/group id
#         usermod  --uid 1000 $USERNAME && \
#         groupmod --gid 1000 $USERNAME

RUN apt update
RUN apt-get update -y
RUN apt-get install -y git \
    curl \
    python3-pip

# Configure ubunru repo
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
# # ROS noetic installation
# RUN apt update
# RUN apt install ros-noetic-desktop-full
# # Dependencies for building packages
# RUN apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# RUN apt install python3-rosdep

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

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
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install gazebo and turtlesim
RUN apt-get update && \
    apt-get install -y ros-noetic-turtlesim && \
    apt-get install -y ros-noetic-gazebo-ros-pkgs
# RUN sudo apt-get install python3-matplotlib
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
COPY requirements.txt ./

# Install requirements
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir -r requirements.txt
RUN rm requirements.txt

WORKDIR /
RUN apt-get update && apt-get install -y \
  wget \
  libglib2.0-0  \
  libgl1-mesa-glx \
  xcb \
  "^libxcb.*" \
  libx11-xcb-dev \
  libglu1-mesa-dev \
  libxrender-dev \
  libxi6 \
  libdbus-1-3 \
  libfontconfig1 \
  xvfb \
  && rm -rf /var/lib/apt/lists/*

# Install VREP
RUN wget https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.tar.xz
ENV vrep=/vrep_sim
RUN mkdir ${vrep}
RUN tar -xf CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.tar.xz -C $vrep
RUN apt-get remove -y wget

RUN echo 'export QT_DEBUG_PLUGINS=1' >> ~/.bashrc
RUN echo 'export PATH=${vrep}:$PATH' >> ~/.bashrc

WORKDIR /catkin_ws

CMD source source devel/setup.bash