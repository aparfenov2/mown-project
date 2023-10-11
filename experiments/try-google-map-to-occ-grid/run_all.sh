#!/bin/bash
ALL_ARGS=("$@")
CONTAINER_NAME="all"
# WORLD="playpen"
# ROBOT="ya_model"
UNKNOWN_ARGS=()
ROSARGS=()
# parameters like rosbridge_port can be specified directly: rosbridge_port:=8081
export TURTLEBOT3_MODEL=burger

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --jetson) JETSON=1;  ;;
        --studio) STUDIO=1 && CONTAINER_NAME="${CONTAINER_NAME}-studio"; ;;
        --sim) ROSARGS+=("sim:=true") && CONTAINER_NAME="${CONTAINER_NAME}-sim";  ;;
        --rviz) ROSARGS+=("rviz:=true") && CONTAINER_NAME="${CONTAINER_NAME}-rviz"; ;;
        --robot) ROBOT="$2"; shift; ;;
        --teleop) ROSARGS+=("teleop:=true") && CONTAINER_NAME="${CONTAINER_NAME}-teleop"; ;;
        --segm) SEGMENTATION=1 && ROSARGS+=("segm:=true") && CONTAINER_NAME="${CONTAINER_NAME}-segm"; ;;
        --segm_net) SEGMENTATION_NET=1 && ROSARGS+=("segm_net:=true") && CONTAINER_NAME="${CONTAINER_NAME}-segm-net"; ;;
        --planning) ROSARGS+=("planning:=true") && CONTAINER_NAME="${CONTAINER_NAME}-planning"; ;;
        --rosbridge) ROSARGS+=("rosbridge:=true") && CONTAINER_NAME="${CONTAINER_NAME}-rosbridge"; ;;
        --command_panel) ROSARGS+=("command_panel:=true") && CONTAINER_NAME="${CONTAINER_NAME}-command-panel"; ;;
        --rtabmap) ROSARGS+=("rtabmap:=true") && CONTAINER_NAME="${CONTAINER_NAME}-rtabmap"; ;;
        --backend) ROSARGS+=("backend:=true") && CONTAINER_NAME="${CONTAINER_NAME}-backend"; ;;

        --pubfix1) PUBFIX=1; CONTAINER_NAME="${CONTAINER_NAME}-pub"; ;;
        --pubfix2) PUBFIX=2; CONTAINER_NAME="${CONTAINER_NAME}-pub"; ;;
        --inner) INNER=1 ;;
        --name) CONTAINER_NAME="${CONTAINER_NAME}-$2"; shift; ;;
        --build) BUILD=1; ;;
        --no_rm) NO_RM="--no_rm";  ;;

        *) echo "roslaunch parameter passed: $1"; UNKNOWN_ARGS+=("$1"); ;;
    esac
    shift
done


[ -n "$INNER" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    export PYTHONPYCACHEPREFIX="/cdir/ws/pycache/"
    export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'

    set -ex
    [ -n "${BUILD}" ] && {
        pushd $PWD
        cd /cdir/ws
        # catkin_make
        catkin config \
        --extend /opt/ros/$ROS_DISTRO \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release
        catkin build
        popd
    }
    . /cdir/ws/devel/setup.bash
    export GAZEBO_MODEL_PATH="/cdir/ws/src/gazebo_models"
    # echo GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}"

    [ "$PUBFIX" == "1" ] && {
        rostopic pub -1 /gps/fix sensor_msgs/NavSatFix "{latitude: 55.154722,longitude: 61.316833}"
        exit 0
    }
    [ "$PUBFIX" == "2" ] && {
        rostopic pub -1 /gps/fix sensor_msgs/NavSatFix "{latitude: 55.154722,longitude: 61.326833}"
        exit 0
    }

    # read robot name from param server if not specified
    rosparam list || {
        roscore &
        sleep 2
    }
    [ -n "$ROBOT" ] && {
        rosparam set robot $ROBOT
    }
    [ -z "$ROBOT" ] && {
        ROBOT=$(rosparam get robot)
    } || true
    [ -n "$ROBOT" ] && {
        ROSARGS+=("robot:=$ROBOT")
    }

    roslaunch /cdir/all.launch ${ROSARGS[@]} ${UNKNOWN_ARGS[@]}
    # roslaunch engix_gazebo engix_playpen.launch
    exit 0
}

# git submodule update --init --recursive  # commented out because it will reset modified submodules to current ver

# setup dependencies
pushd $PWD
cd ws/src

PTH="ddrnet/model/DDRNet_CS.wts"
[ -n "${SEGMENTATION_NET}" ] && {
    [ -f "$PTH" ] || {
        curl -o $PTH  https://kan-rt.ddns.net:8000/ddrnet/DDRNet_CS.wts
    }
}

popd

VOLUMES=()
for f in $(find ws/src -type l); do
    VOLUMES+=("-v $(readlink -f $f):/cdir/$f")
done
VOLUMES+=("-v $(readlink -f all.launch):/cdir/all.launch")
VOLUMES+=("-v $(readlink -f data):/cdir/data")

[ -n "$JETSON" ] && {
    JETSON_ARGS="--ws docker_jetson"
    [ -n "$SEGMENTATION" ] && {
        JETSON_ARGS="--ws docker_jetson_ml"
        apt -qq list nvidia-container-csv-tensorrt 2>/dev/null | grep -qE "(installed|upgradeable)" || {
            echo please apt install nvidia-container-csv-tensorrt
            exit 1
        }
    }
}

[ -n "$STUDIO" ] && {
[ -n "${BUILD}" ] && {
    cd studio
    docker build -t foxglove:my .
}
docker run --rm \
    ${VOLUMES[@]} \
    --name ${CONTAINER_NAME} \
    -p "8080:8080" \
    foxglove:my

    exit 0
}

mkdir .ros || true
bash _run_in_docker.sh ${JETSON_ARGS} ${NO_RM} --script $0 --name ${CONTAINER_NAME} \
    -v $PWD/.ros:/root/.ros \
    ${VOLUMES[@]} \
    ${ALL_ARGS[@]}
