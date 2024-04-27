#!/bin/bash
source .env

IMAGE_NAME="$IMAGE_NAME:$TAG"

DOCKER_PATH="$PWD/../../docker/${ARTIFACT_NAME}"

docker build -t $IMAGE_NAME -f ../../docker/${ARTIFACT_NAME}/Dockerfile $DOCKER_PATH
