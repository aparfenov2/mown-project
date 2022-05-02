#!/bin/bash
DOCKER_IMAGE_NAME=$(cat docker/image)
DOCKER_CONTAINER_NAME=$(cat docker/image)
docker stop $DOCKER_CONTAINER_NAME || true && docker rm $DOCKER_CONTAINER_NAME || true

docker build -t $(cat docker/image) docker/
SCRIPTS_FOLDER='../scripts'

VOLUMES=()
for f in $(find ws/src -type l); do
	ff=${f:7}
    VOLUMES+=("-v $(readlink -f $f):/catkin_ws/src/$ff")
done

for file in "$SCRIPTS_FOLDER"/*; do
	ff=${file:11}
	chmod +x $file
	VOLUMES+=("-v $(readlink -f $file):/usr/bin/$ff")

done
sudo xhost +local:root

docker run \
	-it \
	-p 8088:8088 \
	--name $DOCKER_CONTAINER_NAME \
	--gpus all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	${VOLUMES[@]} \
    $DOCKER_IMAGE_NAME bash
