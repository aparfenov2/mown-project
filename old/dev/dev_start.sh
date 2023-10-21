#!/bin/bash
COMMON_DOCKER_IMAGE_NAME=$(cat docker/image)
DEV_DOCKER_IMAGE_NAME=dev-$COMMON_DOCKER_IMAGE_NAME
DOCKER_CONTAINER_NAME=$DEV_DOCKER_IMAGE_NAME

if [ "$( docker container inspect -f '{{.State.Running}}' $DOCKER_CONTAINER_NAME )" == "false" ]; then
    echo Start $DOCKER_CONTAINER_NAME ...
    xhost +local:root
    docker start $DOCKER_CONTAINER_NAME
fi

docker exec -ti $DOCKER_CONTAINER_NAME bash