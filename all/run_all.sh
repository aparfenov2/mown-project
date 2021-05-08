#!/bin/bash
SEGMENTATION_BYPASS=1
ROSCORE=1
ALL_ARGS=("$@")

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --inner) INNER=1 ;;
        --sim) SIM=1;  ;;
        --rviz) RVIZ=1; ;;
        --teleop) TELEOP=1; ;;
        --roscore) ROSCORE=1 ;;
        ---roscore) ROSCORE="" ;;
        --segm) SEGMENTATION=1; ;;
        --mb_mod) MOVE_BASE_MOD=1; ;;
        ---segm_bypass) SEGMENTATION_BYPASS=""; ;;
        --proj) PROJECTION=1; ;;
        --loca) LOCALIZATION=1; ;;
        --sim_basic) SIM=1; RVIZ=1; TELEOP=1; ;;
        --preset1) SIM=1; RVIZ=1; TELEOP=1; SEGMENTATION=1; PROJECTION=1; LOCALIZATION=1;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

ROSARGS=()
[ -n "$SIM" ] && ROSARGS+=("sim:=true")
[ -n "$RVIZ" ] && ROSARGS+=("rviz:=true")
[ -n "$TELEOP" ] && ROSARGS+=("teleop:=true")
[ -n "$SEGMENTATION" ] && ROSARGS+=("segm:=true")
[ -n "$SEGMENTATION_BYPASS" ] && ROSARGS+=("segm_bypass:=true")
[ -n "$PROJECTION" ] && ROSARGS+=("proj:=true")
[ -n "$LOCALIZATION" ] && ROSARGS+=("loca:=true")
[ -n "$MOVE_BASE_MOD" ] && ROSARGS+=("mb_mod:=true")

[ -n "$INNER" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
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
    roslaunch my_utils_common all.launch ${ROSARGS[@]}
    exit 0
}

# setup dependencies
pushd $PWD
cd ws/src
# sim
[ -d "velodyne_simulator" ] || {
    git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
}
pushd $PWD
cd velodyne_simulator
git checkout 1.0.9
popd

# segm
[ -d "catkin_simple" ] || {
    git clone git@github.com:catkin/catkin_simple.git
}
PTH="segmentation_node/model/fcn_hr18s_512x1024_40k_cityscapes_20200601_014216-93db27d0.pth"
[ -n "${SEGMENTATION_BYPASS}" ] && {
    [ -f "$PTH" ] || {
        curl -o $PTH  https://download.openmmlab.com/mmsegmentation/v0.5/hrnet/fcn_hr18s_512x1024_40k_cityscapes/fcn_hr18s_512x1024_40k_cityscapes_20200601_014216-93db27d0.pth
    }
}

# loca
[ -d "A-LOAM" ] || {
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    # echo make build > A-LOAM/docker/build.sh
}

# proj
[ -d "elevation_mapping" ] || {
    git clone git@github.com:ANYbotics/elevation_mapping.git
}
[ -d "grid_map" ] || {
    git clone git@github.com:ANYbotics/grid_map.git
}
[ -d "kindr" ] || {
    git clone git@github.com:ANYbotics/kindr.git
}
[ -d "kindr_ros" ] || {
    git clone git@github.com:ANYbotics/kindr_ros.git
}

popd

VOLUMES=()
for f in $(find ws/src -type l); do
    VOLUMES+=("-v $(readlink -f $f):/cdir/$f")
done

_NAME="all"
[ -n "$TELEOP" ] && {
    _NAME="teleop"
}

bash _run_in_docker.sh --script $0 --name ${_NAME} \
    ${VOLUMES[@]} \
    ${ALL_ARGS[@]}
