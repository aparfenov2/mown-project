# ------- ROS core --------

FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y ros-noetic-octomap-msgs \
    ros-noetic-rosbridge-suite \
    python3-pip \
    python3-catkin-tools

RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc

WORKDIR /catkin_ws

COPY run_inside.sh .

