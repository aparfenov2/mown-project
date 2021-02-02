#!/bin/bash
set -ex

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
    *)    # unknown option
    shift # past argument
    ;;
esac
done

[ -z "$NAME" ] || [ -z "$SCRIPT" ] && {
    echo "required parameters not set"
    exit 1
}

DOCKERWS="."
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
    --name "$NAME" \
    -v ${PWD}:/cdir \
    -v ~/.gazebo/models:/root/.gazebo/models \
    -w /cdir \
    $IMAGE bash "$SCRIPT" --inner
