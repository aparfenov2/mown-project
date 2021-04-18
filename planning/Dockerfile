FROM ros:noetic

RUN apt update
RUN apt-get update -y
RUN apt-get install -y git \
    curl \
    python3-pip
# RUN sudo apt-get install python3-matplotlib
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
COPY requirements.txt ./

RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir -r requirements.txt