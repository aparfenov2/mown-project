#!/bin/bash
container_name=$(cat docker/image)

if [ "$( docker container inspect -f '{{.State.Running}}' $container_name )" == "false" ]; then
    echo Start $container_name ...
    xhost +local:root
    docker start $container_name
fi

docker exec -ti $container_name bash