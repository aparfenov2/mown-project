#!/bin/bash
source .env

SIM_VOLUMES="$PWD/../../modules/simulator:/catkin_ws/src/simulator"
GAZEBO_MODELS_VOLUME="$PWD/../../3rd_party/gazebo_models:/catkin_ws/src/gazebo_models"
GAZEBO_SIM_VOLUME="$PWD/../../3rd_party/gazebo_sim:/catkin_ws/src/gazebo_sim"
VELODYNE_SIM_VOLUME="$PWD/../../3rd_party/velodyne_simulator:/catkin_ws/src/velodyne_simulator"
SCRIPTS_VOLUME="$PWD/scripts/:/catkin_ws/scripts"
ENGIX_DESC_VOLUME="$PWD/../../modules/core/engix_description/:/catkin_ws/src/engix_description"
ENGIX_ROBOT_VOLUME="$PWD/../../modules/core/engix_robot/:/catkin_ws/src/engix_robot/"

echo Run container

xhost +local:root
docker run \
	-d \
	-it \
	--net=host \
	--name $CONTAINER_NAME \
	--gpus all \
	--volume /dev:/dev \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-v $SIM_VOLUMES \
	-v $SCRIPTS_VOLUME \
	-v $GAZEBO_MODELS_VOLUME \
	-v $GAZEBO_SIM_VOLUME \
	-v $VELODYNE_SIM_VOLUME \
	-v $ENGIX_DESC_VOLUME \
	-v $ENGIX_ROBOT_VOLUME \
    $IMAGE_NAME

echo Build ws inside container

docker exec $CONTAINER_NAME bash -c /catkin_ws/scripts/build.sh

echo Run sim

xhost +local:root
docker exec $CONTAINER_NAME bash -c /catkin_ws/scripts/run.sh
