#!/bin/bash
COMMON_DOCKER_IMAGE_NAME=$(cat docker/image)
DEV_DOCKER_IMAGE_NAME=dev-$COMMON_DOCKER_IMAGE_NAME
DOCKER_CONTAINER_NAME=$DEV_DOCKER_IMAGE_NAME

echo "###############################"
echo "## Try to stop container $DOCKER_CONTAINER_NAME"
docker stop $DOCKER_CONTAINER_NAME || true && docker rm $DOCKER_CONTAINER_NAME || true

echo "## Build container $DOCKER_CONTAINER_NAME"

docker build -t $COMMON_DOCKER_IMAGE_NAME docker/
docker build -t $DEV_DOCKER_IMAGE_NAME -f docker/Dockerfile:dev docker/
SCRIPTS_FOLDER='../scripts'

WS_DIR='/catkin_ws'
PROJECT_DIR='/mown-project'

VOLUMES=()
# for f in $(find src -type l); do
# 	ff=${f:7}
#     VOLUMES+=("-v $(readlink -f $f):$WS_DIR/src/$ff")
# done

# for file in "$SCRIPTS_FOLDER"/*; do
# 	ff=${file:11}
# 	chmod +x $file
# 	VOLUMES+=("-v $(readlink -f $file):/usr/bin/$ff")

# done

# PROJECT_FOLDER='../'
# for file in "$PROJECT_FOLDER"/*; do
# 	ff=${file:11}
# 	chmod +x $file
# 	VOLUMES+=("-v $(readlink -f $file):/usr/bin/$ff")

# done
PROJECT_VOLUME="$(pwd)/../../mown-project:/$PROJECT_DIR"
VOLUMES+=("$(pwd)/../../mown-project:/mown-project")

echo "## Run container $DOCKER_CONTAINER_NAME"
docker run \
	-d \
	-it \
	-p 8088:8088 \
	--name $DOCKER_CONTAINER_NAME \
	--gpus all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-v $PROJECT_VOLUME \
    $DEV_DOCKER_IMAGE_NAME bash
echo ""
echo ""

docker exec $DOCKER_CONTAINER_NAME bash -c $PROJECT_DIR'/dev/run_inside_container.sh'

echo ""
echo ""
echo "## Build done "
echo "###############################"
# docker start $DOCKER_CONTAINER_NAME

# docker exec $DOCKER_CONTAINER_NAME run_inside_container.sh
# sudo xhost +local:root

# docker run \
# 	-it \
# 	-p 8088:8088 \
# 	--name $DOCKER_CONTAINER_NAME \
# 	--gpus all \
# 	-v /tmp/.X11-unix:/tmp/.X11-unix \
# 	-e DISPLAY=$DISPLAY \
# 	-e QT_X11_NO_MITSHM=1 \
# 	${VOLUMES[@]} \
#     $DOCKER_IMAGE_NAME bash
