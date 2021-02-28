set -e

OTHERS=()
POSITIONAL=("$@")

while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --inner)
    IN_DOCKER=1
    shift # past argument
    ;;
    --build)
    BUILD=1
    shift # past argument
    ;;
    --name)
    NAME="$2"
    shift # past argument
    shift # past argument
    ;;
    *)    # unknown option
    OTHERS+=($1)
    shift # past argument
    ;;
esac
done

[ -z "${OTHERS[@]}" ] && {
    echo specify command or launch file name
    exit 1
}

echo OTHERS="${OTHERS[@]}"

[ -n "${IN_DOCKER}" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    set -ex
    [ -n "$BUILD" ] && {
        cd /cdir/utils_ws/
        catkin_make
    }
    . /cdir/utils_ws/devel/setup.sh
    roslaunch my_utils_common "${OTHERS[@]}"
    exit 0
}

[ -n "$NAME" ] && {
    _NAME="--name ${NAME}"
}

bash _run_in_docker.sh --script $0 ${_NAME} \
    -v $(readlink -f utils_ws/src/my_utils_common):/cdir/utils_ws/src/my_utils_common \
    "${POSITIONAL[@]}"
