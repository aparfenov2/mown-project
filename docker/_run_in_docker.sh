#!/bin/bash
set -e
VOLUMES=()
OTHERS=()
DOCKERWS="docker"
BUILD_IMAGE=1

while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --name)
    NAME="$2"
    shift # past argument
    shift # past value
    ;;
    --build_image)
    BUILD_IMAGE=1
    shift # past argument
    ;;
    --image)
    IMAGE="$1"
    shift # past argument
    shift # past argument
    ;;
    --ws)
    DOCKERWS="$1"
    shift # past argument
    shift # past argument
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
    OTHERS+=($1)
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

[ -z "$IMAGE" ] && {
    IMAGE=$(cat $DOCKERWS/image)
}

echo DOCKERWS=$DOCKERWS
echo IMAGE=$IMAGE
echo OTHERS="${OTHERS[@]}"

[ -n "${BUILD_IMAGE}" ] && {
    pushd $PWD
    cd $DOCKERWS && bash ./build.sh
    popd
}

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
    $IMAGE bash /tmp/_outer.sh --inner "${OTHERS[@]}"
