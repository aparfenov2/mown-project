#!/bin/bash
source .env

IMAGE_NAME="$IMAGE_NAME:$TAG"


SCRIPTS_VOLUMES="$PWD/../../scripts/core-dev-x86/:/catkin_ws/scripts/"
THIRD_PARTY_VOLUMES="$PWD/../../3rd_party/:/catkin_ws/src/3rd_party"
WORK_SPASE_VOLUMES="$PWD/../../modules:/catkin_ws/src"

docker run \
	-d \
	-it \
	--net=host \
	--name $CONTAINER_NAME \
	--gpus all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-v $SCRIPTS_VOLUMES \
	-v $WORK_SPASE_VOLUMES \
	-v $THIRD_PARTY_VOLUMES \
    $IMAGE_NAME

docker exec $CONTAINER_NAME bash -c /catkin_ws/scripts/run_inside_container.sh
