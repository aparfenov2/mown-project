#!/bin/bash
SEGMENTATION_BYPASS=1
ROSCORE=1
ALL_ARGS=("$@")
CONTAINER_NAME="all"

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
                --name) CONTAINER_NAME="$2"; shift; ;;
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
[ -n "$SIM" ] && ROSARGS+=("sim:=true") && CONTAINER_NAME="sim"
[ -n "$RVIZ" ] && ROSARGS+=("rviz:=true") && CONTAINER_NAME="rviz"
[ -n "$TELEOP" ] && ROSARGS+=("teleop:=true") && CONTAINER_NAME="teleop"
[ -n "$SEGMENTATION" ] && ROSARGS+=("segm:=true") && CONTAINER_NAME="segm"
[ -n "$SEGMENTATION_BYPASS" ] && ROSARGS+=("segm_bypass:=true")
[ -n "$PROJECTION" ] && ROSARGS+=("proj:=true") && CONTAINER_NAME="proj"
[ -n "$LOCALIZATION" ] && ROSARGS+=("loca:=true") && CONTAINER_NAME="loca"
[ -n "$MOVE_BASE_MOD" ] && ROSARGS+=("mb_mod:=true") && CONTAINER_NAME="mb_mod"

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

git submodule update --init --recursive

# setup dependencies
pushd $PWD
cd ws/src
# # sim
# [ -d "velodyne_simulator" ] || {
#     git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
# }
# pushd $PWD
# cd velodyne_simulator
# git checkout 1.0.9
# popd

# # segm
# [ -d "catkin_simple" ] || {
#     git clone git@github.com:catkin/catkin_simple.git
# }
# PTH="segmentation_node/model/fcn_hr18s_512x1024_40k_cityscapes_20200601_014216-93db27d0.pth"
PTH="ddrnet/model/DDRNet_CS.wts"
[ -n "${SEGMENTATION_BYPASS}" ] && {
    [ -f "$PTH" ] || {
        # curl -o $PTH  https://download.openmmlab.com/mmsegmentation/v0.5/hrnet/fcn_hr18s_512x1024_40k_cityscapes/fcn_hr18s_512x1024_40k_cityscapes_20200601_014216-93db27d0.pth
        curl -o $PTH  https://kan-rt.ddns.net:8000/DDRNet_CS.wts
    }
}

# # loca
# [ -d "A-LOAM" ] || {
#     git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
#     # echo make build > A-LOAM/docker/build.sh
# }

# # proj
# [ -d "elevation_mapping" ] || {
#     git clone git@github.com:ANYbotics/elevation_mapping.git
# }
# [ -d "grid_map" ] || {
#     git clone git@github.com:ANYbotics/grid_map.git
# }
# [ -d "kindr" ] || {
#     git clone git@github.com:ANYbotics/kindr.git
# }
# [ -d "kindr_ros" ] || {
#     git clone git@github.com:ANYbotics/kindr_ros.git
# }

popd

VOLUMES=()
for f in $(find ws/src -type l); do
    VOLUMES+=("-v $(readlink -f $f):/cdir/$f")
done

bash _run_in_docker.sh --script $0 --name ${CONTAINER_NAME} \
    ${VOLUMES[@]} \
    ${ALL_ARGS[@]}
