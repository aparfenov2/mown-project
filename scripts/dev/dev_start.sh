#!/bin/bash
source .env


if [ "$( docker container inspect -f '{{.State.Running}}' $CONTAINER_NAME )" == "false" ]; then
    echo Start $CONTAINER_NAME ...
    xhost +local:root
    docker start $CONTAINER_NAME
fi

echo $CONTAINER_NAME

docker exec -ti $CONTAINER_NAME bash