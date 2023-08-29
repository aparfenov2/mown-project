#!/bin/bash
source .env


IMAGE_NAME="$IMAGE_NAME-dev:$TAG"


function dev_build() {
    DOCKER_PATH="$PWD/../../docker"


    echo "###############################"
    echo "## Try to stop container $CONTAINER_NAME"
    docker stop $CONTAINER_NAME || true && docker rm $CONTAINER_NAME || true

    echo "## Build container $CONTAINER_NAME"

    docker build -t $IMAGE_NAME -f ../../docker/Dockerfile $DOCKER_PATH
    docker build -t $IMAGE_NAME -f ../../docker/Dockerfile.dev $DOCKER_PATH

    echo ""
    echo ""
    echo "## Build done "
    echo "###############################"
}

function dev_start() {
    if [ "$( docker container inspect -f '{{.State.Running}}' $CONTAINER_NAME )" == "false" ]; then
        echo Start $CONTAINER_NAME ...
        xhost +local:root
        docker start $CONTAINER_NAME
    fi


    docker exec -ti $CONTAINER_NAME bash
}

function dev_run() {
    PROJECT_DIR='/mown-project'

    SCRIPTS_VOLUMES="$PWD/../:$PROJECT_DIR/scripts"
    WORK_SPASE_VOLUMES="$PWD/../../engix:$PROJECT_DIR/src"

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

    docker exec $CONTAINER_NAME bash -c $PROJECT_DIR/scripts/dev/run_inside_container.sh
}

function dev_rm() {
    docker rm $CONTAINER_NAME
}

function dev_stop() {
    docker stop $CONTAINER_NAME
}

ARGUMENT=$1

case $ARGUMENT in
--build)
    dev_build
    ;;

--start)
    dev_start
    ;;

--join)
    dev_start
    ;;

--run)
    dev_run
    ;;

--rm)
    dev_rm
    ;;

--stop)
    dev_stop
    ;;

*)
    echo "Wrong argument. Usage: ws_build [--build | --start | --join | --run | --rm | --stop]"
    ;;
esac