#!/bin/bash
source .env

DOCKER_PATH="$PWD/../../docker/navigation-arm64"

docker build -t $IMAGE_NAME -f ../../docker/navigation-arm64/Dockerfile $DOCKER_PATH
