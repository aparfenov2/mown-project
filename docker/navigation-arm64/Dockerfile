# ------- ROS core --------

FROM ros:noetic-ros-core


RUN apt-get update && apt-get install -y ros-noetic-octomap-msgs \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    ros-noetic-tf

WORKDIR /tmp

COPY requirements.txt ./

# Install requirements
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir -r requirements.txt
RUN rm requirements.txt

RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc

WORKDIR /catkin_ws

# bootstrap rosdep
RUN rosdep init && rosdep update
