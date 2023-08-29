#!/bin/bash
source .env

IMAGE_NAME="$IMAGE_NAME:$TAG"

DOCKER_PATH="$PWD/../../docker"

docker build -t $IMAGE_NAME -f ../../docker/Dockerfile $DOCKER_PATH

# PROJECT_DIR='/catkin_ws'

# WORK_SPACE_VOLUMES="$PWD/../../engix:/catkin_ws/src"
# THIRD_PARTY_VOLUMES="$PWD/../../3rd_party:/catkin_ws/src/3rd_party"
# SCRIPTS_VOLUMES="$PWD/scripts:/catkin_ws/scripts"

# echo "## Run container $CONTAINER_NAME"
# docker run \
# 	-d \
# 	-it \
# 	--net=host \
# 	--name $CONTAINER_NAME \
# 	--gpus all \
# 	-v /tmp/.X11-unix:/tmp/.X11-unix \
# 	-e DISPLAY=$DISPLAY \
# 	-e QT_X11_NO_MITSHM=1 \
# 	-v $WORK_SPACE_VOLUMES \
# 	-v $THIRD_PARTY_VOLUMES \
# 	-v $SCRIPTS_VOLUMES \
#     $IMAGE_NAME

# xhost +local:root
# docker exec -ti ${CONTAINER_NAME} bash -c 'cd /catkin_ws/scripts && ./run.sh'

