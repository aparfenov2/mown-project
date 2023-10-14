#!/bin/bash
source .env

SCRIPTS_VOLUMES="$PWD/../../scripts/planning-arm64/:/catkin_ws/scripts/"
ENGIX_MSGS_VOLUMES="$PWD/../../modules/msgs/:/catkin_ws/src/msgs"
NAVIGATION_VOLUMES="$PWD/../../modules/navigation:/catkin_ws/src/navigation"
LIB_VOLUMES="$PWD/../../modules/libs:/catkin_ws/src/libs"

docker run \
	-d \
	-it \
	--net=host \
	--name $CONTAINER_NAME \
	-v $SCRIPTS_VOLUMES \
	-v $ENGIX_MSGS_VOLUMES \
	-v $NAVIGATION_VOLUMES \
	-v $LIB_VOLUMES \
    $IMAGE_NAME

docker exec $CONTAINER_NAME bash -c /catkin_ws/scripts/run_inside_container.sh
