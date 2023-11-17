#!/bin/bash
source .env

DOCKER_PATH="$PWD/../../docker/gazebo"

docker build  --no-cache -t $IMAGE_NAME -f $PWD/../../docker/gazebo/Dockerfile $DOCKER_PATH
