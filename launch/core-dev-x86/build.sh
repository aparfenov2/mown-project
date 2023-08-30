#!/bin/bash
source .env

IMAGE_NAME="$IMAGE_NAME:$TAG"

DOCKER_PATH="$PWD/../../docker/core-dev-x86"

docker build -t $IMAGE_NAME -f ../../docker/core-dev-x86/Dockerfile $DOCKER_PATH
