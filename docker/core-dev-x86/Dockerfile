# ------- ROS core --------

FROM kan-rt.ddns.net:8929/docker/ros-mower:mower-gui

# ANYbotics/elevation_mapping
RUN apt-get update && apt-get install -y ros-noetic-octomap-msgs

RUN apt-get update && apt-get install -y python3-pip


WORKDIR /tmp

COPY requirements.txt ./

# Install requirements
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir -r requirements.txt
RUN rm requirements.txt

RUN apt-get install -y python3-tk
RUN python3 -m pip install -U catkin_tools

ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

RUN echo 'source ${ROS_ROOT}/setup.bash' >> /root/.bashrc

WORKDIR /catkin_ws

RUN pip3 install osrf-pycommon

# debug support
RUN apt-get update && apt-get install -y xterm gdb

# color-picker
# RUN sudo ln -s $(which python3) /usr/bin/python
# RUN pip3 install scipy

RUN apt-get update && apt-get install -y ros-noetic-pointcloud-to-laserscan ros-noetic-ros-numpy
# RUN apt-get update && apt-get install -y ros-noetic-move-base-flex

# EKF
RUN apt-get update && apt-get install -y ros-noetic-robot-localization
# message_to_tf
# RUN apt-get update && apt-get install -y ros-noetic-hector-localization

# gazebo GPS
RUN apt-get update && apt-get install -y ros-noetic-hector-gazebo-plugins

RUN apt-get install -y ros-noetic-rosbridge-suite ros-noetic-foxglove-bridge
RUN apt-get update && apt-get install -y ros-noetic-rtabmap-ros

RUN apt-get update && apt-get install -y libturbojpeg
RUN ln -s $(which python3) /usr/bin/python

# RUN add-apt-repository ppa:deadsnakes/ppa
# RUN apt-get update
RUN apt-get install python3-tk
RUN python3 -m pip install Rtree
RUN python3 -m pip install casadi
RUN python3 -m pip install msgpack-rpc-python
# RUN apt-get update && apt-get install -y ros-noetic-robot-pose-ekf
# RUN apt-get update && apt-get install ros-noetic-depthimage-to-laserscan
