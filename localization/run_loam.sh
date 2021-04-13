ALL="$@"
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --inner) INNER=1 ;;
        *) echo unrocognized $1; exit 1 ;;
    esac
    shift
done

[ -n "$INNER" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    set -e
    pushd $PWD
    cd ws
    catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release
    
    catkin build
    popd

    . ws/devel/setup.bash

    # cd /root/catkin_ws/
    # source devel/setup.bash
    # source /cdir/utils_ws/devel/setup.bash

    # roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch rviz:=false
    roslaunch my_utils_common aloam.launch

    exit 0
}

pushd $PWD
cd ws/src/
[ -d "A-LOAM" ] || {
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    # echo make build > A-LOAM/docker/build.sh
}
popd

bash _run_in_docker.sh --script $0 \
    -v $(readlink -f ws):/cdir/ws \
    -v $(readlink -f ws/src/my_utils_common):/cdir/ws/src/my_utils_common \
    $ALL

exit 0
