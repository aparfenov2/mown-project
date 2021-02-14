#!/bin/bash
set -ex
VOLUMES=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --name)
    NAME="$2"
    shift # past argument
    shift # past value
    ;;
    --script)
    SCRIPT="$2"
    shift # past argument
    shift # past value
    ;;
    -v|--volume)
    VOLUMES+=(-v $2)
    shift # past argument
    shift # past value
    ;;
    *)    # unknown option
    shift # past argument
    ;;
esac
done

[ -z "$SCRIPT" ] && {
    echo "required parameters not set"
    exit 1
}

[ -n "$NAME" ] && {
    _NAME="--name $NAME"
}

DOCKERWS="docker"
IMAGE=ros-mower

echo DOCKERWS=$DOCKERWS
echo IMAGE=$IMAGE

_pwd=$PWD
cd $DOCKERWS && bash ./build.sh
cd ${_pwd}

xhost +
docker run -ti --rm \
    --gpus all \
    -e "DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e XAUTHORITY \
    -v /dev:/dev \
    --privileged \
    --net=host \
    ${_NAME} \
    -v ${PWD}:/cdir \
    -v $(readlink -f $SCRIPT):/tmp/_outer.sh \
    -v ~/.gazebo/models:/root/.gazebo/models \
    ${VOLUMES[@]} \
    -w /cdir \
    $IMAGE bash /tmp/_outer.sh --inner
