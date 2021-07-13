#!/bin/bash
SEGMENTATION_BYPASS=1
ALL_ARGS=("$@")
CONTAINER_NAME="all"
WORLD="turtletown"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --inner) INNER=1 ;;
        --sim) SIM=1;  ;;
        --grass_world) WORLD="baylands";  ;;
        --world) WORLD="$2"; shift; ;;
        --rviz) RVIZ=1; ;;
        --teleop) TELEOP=1; ;;
        --segm) SEGMENTATION=1; ;;
        --mb) MOVE_BASE=1; ;;
        --name) CONTAINER_NAME="$2"; shift; ;;
        ---segm_bypass) SEGMENTATION_BYPASS=""; ;;
        --loca) LOCALIZATION=1; ;;
        # --sim_basic) SIM=1; RVIZ=1; TELEOP=1; ;;
        # --preset1) SIM=1; RVIZ=1; TELEOP=1; SEGMENTATION=1; PROJECTION=1; LOCALIZATION=1;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

ROSARGS=()
[ -n "$SIM" ] && ROSARGS+=("sim:=true") && CONTAINER_NAME="sim"
[ -n "$WORLD" ] && ROSARGS+=("world:=$WORLD")
[ -n "$RVIZ" ] && ROSARGS+=("rviz:=true") && CONTAINER_NAME="rviz"
[ -n "$TELEOP" ] && ROSARGS+=("teleop:=true") && CONTAINER_NAME="teleop"
[ -n "$SEGMENTATION" ] && ROSARGS+=("segm:=true") && CONTAINER_NAME="segm"
[ -n "$SEGMENTATION_BYPASS" ] && ROSARGS+=("segm_bypass:=true")
[ -n "$PROJECTION" ] && ROSARGS+=("proj:=true") && CONTAINER_NAME="proj"
[ -n "$LOCALIZATION" ] && ROSARGS+=("loca:=true") && CONTAINER_NAME="loca"
[ -n "$MOVE_BASE" ] && ROSARGS+=("mb:=true") && CONTAINER_NAME="mb"

[ -n "$INNER" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    export PYTHONPYCACHEPREFIX="/cdir/pycache/"

    set -ex
    pushd $PWD
    cd /cdir/ws 
    # catkin_make
    catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release
    catkin build
    popd

    . /cdir/ws/devel/setup.bash
    export GAZEBO_MODEL_PATH="/cdir/ws/src/gazebo_models"
    # echo GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}"
    roslaunch my_utils_common all.launch ${ROSARGS[@]}
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

bash _run_in_docker.sh --script $0 --name ${CONTAINER_NAME} \
    ${VOLUMES[@]} \
    ${ALL_ARGS[@]}
