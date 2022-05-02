#!/bin/bash
ALL_ARGS=("$@")
CONTAINER_NAME="all"
WORLD="playpen"
ROBOT="ya_model"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --inner) INNER=1 ;;
        --jetson) JETSON=1;  ;;
        --no_rm) NO_RM="--no_rm";  ;;
        --sim) SIM=1;  ;;
        --grass_world) WORLD="baylands";  ;;
        --world) WORLD="$2"; shift; ;;
        --paused) PAUSED=1; ;;
        --rviz) RVIZ=1; ;;
        --robot) ROBOT="$2"; shift; ;;
        --robot_ya) ROBOT="ya_model"; ;;
        --robot_turtle) ROBOT="turtlebot"; ;;
        --teleop) TELEOP=1; ;;
        --segm) SEGMENTATION=1; ;;
        --segm_net) SEGMENTATION_NET=1; ;;
        --mb) MOVE_BASE=1; ;;
        --name) CONTAINER_NAME="$2"; shift; ;;
        --loca) LOCALIZATION=1; ;;
        --planning) PLANNING=1; ;;
        --rosbridge) ROSBRIDGE=1; ;;
        --rosbridge_port) ROSBRIDGE_PORT="$2"; shift; ;;
        --command_panel) COMMAND_PANEL=1; ;;
        --no_build) NO_BUILD=1; ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

ROSARGS=()
[ -n "$SIM" ] && ROSARGS+=("sim:=true") && CONTAINER_NAME="sim"
[ -n "$WORLD" ] && ROSARGS+=("world:=$WORLD")
[ -n "$PAUSED" ] && ROSARGS+=("paused:=true")
[ -n "$RVIZ" ] && ROSARGS+=("rviz:=true") && CONTAINER_NAME="rviz"
[ -n "$TELEOP" ] && ROSARGS+=("teleop:=true") && CONTAINER_NAME="teleop"
[ -n "$SEGMENTATION" ] && ROSARGS+=("segm:=true") && CONTAINER_NAME="segm"
[ -n "${SEGMENTATION_NET}" ] && ROSARGS+=("segm_net:=true") && CONTAINER_NAME="segm_net"
[ -n "$ROBOT" ] && ROSARGS+=("robot:=$ROBOT")
[ -n "$PROJECTION" ] && ROSARGS+=("proj:=true") && CONTAINER_NAME="proj"
[ -n "$LOCALIZATION" ] && ROSARGS+=("loca:=true") && CONTAINER_NAME="loca"
[ -n "$MOVE_BASE" ] && ROSARGS+=("mb:=true") && CONTAINER_NAME="mb"
[ -n "$PLANNING" ] && ROSARGS+=("planning:=true") && CONTAINER_NAME="planning"
[ -n "$ROSBRIDGE" ] && ROSARGS+=("rosbridge:=true") && CONTAINER_NAME="rosbridge"
[ -n "$ROSBRIDGE_PORT" ] && ROSARGS+=("rosbridge_port:=$ROSBRIDGE_PORT")
[ -n "$COMMAND_PANEL" ] && ROSARGS+=("command_panel:=true") && CONTAINER_NAME="command_panel"

[ -n "$INNER" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    export PYTHONPYCACHEPREFIX="/cdir/ws/pycache/"

    set -ex
    [ -z "${NO_BUILD}" ] && {
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
    roslaunch my_utils_common all.launch ${ROSARGS[@]}
    # roslaunch engix_gazebo engix_playpen.launch
    exit 0
}

git submodule update --init --recursive

# setup dependencies
pushd $PWD
cd ws/src

PTH="ddrnet/model/DDRNet_CS.wts"
[ -n "${SEGMENTATION_BYPASS}" ] && {
    [ -f "$PTH" ] || {
        curl -o $PTH  https://kan-rt.ddns.net:8000/ddrnet/DDRNet_CS.wts
    }
}

popd

VOLUMES=()
for f in $(find ws/src -type l); do
    VOLUMES+=("-v $(readlink -f $f):/cdir/$f")
done

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

bash _run_in_docker.sh ${JETSON_ARGS} ${NO_RM} --script $0 --name ${CONTAINER_NAME} \
    ${VOLUMES[@]} \
    ${ALL_ARGS[@]}
