
set -ex

[ "$1" == "--inner" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    # . /cdir/simulator_ws/devel/setup.sh
    cd /cdir/simulator_ws && catkin_make
    exit 0
}

bash _run_in_docker.sh --script $0 --name kobuki-sim
