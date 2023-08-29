#!/bin/bash
source .env

IMAGE_NAME="$IMAGE_NAME:$TAG"

DOCKER_PATH="$PWD/../../docker"

docker build -t $IMAGE_NAME -f ../../docker/Dockerfile $DOCKER_PATH
docker build -t $IMAGE_NAME -f ../../docker/Dockerfile.dev $DOCKER_PATH


PROJECT_DIR='/mown-project'

SCRIPTS_VOLUMES="$PWD/../:$PROJECT_DIR/scripts/dev/scripts"
WORK_SPASE_VOLUMES="$PWD/../../engix:$PROJECT_DIR/src"

echo "## Run container $CONTAINER_NAME"
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
    $IMAGE_NAME

echo ""
echo ""

docker exec $CONTAINER_NAME bash -c $PROJECT_DIR/scripts/dev/run_inside_container.sh

echo ""
echo ""
echo "## Build done "
echo "###############################"
