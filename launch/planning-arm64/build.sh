#!/bin/bash
source .env

DOCKER_PATH="$PWD/../../docker/planning-arm64"

docker build -t $IMAGE_NAME -f ../../docker/planning-arm64/Dockerfile $DOCKER_PATH
