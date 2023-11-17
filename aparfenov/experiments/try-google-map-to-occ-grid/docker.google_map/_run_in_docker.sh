#!/bin/bash
set -e
VOLUMES=()
ENVT=()
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
    --no_rm)
    NO_RM=1
    shift # past argument
    ;;
    --build_image)
    BUILD_IMAGE=1
    shift # past argument
    ;;
    --image)
    IMAGE="$2"
    shift # past argument
    shift # past argument
    ;;
    --ws)
    DOCKERWS="$2"
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
    -e|--environment)
    ENVT+=(-e $2)
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

[ -z "${NO_RM}" ] && {
    _RM="--rm"
}

xhost +
docker run -ti ${_RM} \
    --gpus all \
    -e "DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "NVIDIA_DRIVER_CAPABILITIES=all" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ${ENVT[@]} \
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
    --entrypoint "" \
    $IMAGE bash /tmp/_outer.sh --inner "${OTHERS[@]}"
