
[ "$1" == "--inner" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    set -ex
    pushd $PWD
    cd /cdir/simulator_ws && catkin_make
    popd
    . /cdir/simulator_ws/devel/setup.sh
    roslaunch tb_gazebo turtletown.launch
    exit 0
}

bash _run_in_docker.sh --script $0 --name gazebo_sim \
    -v $(readlink -f simulator_ws/src/tb_gazebo):/cdir/simulator_ws/src/tb_gazebo \
    -v $(readlink -f simulator_ws/src/tb_gazebo_description):/cdir/simulator_ws/src/tb_gazebo_description \
    -v $(readlink -f simulator_ws/src/tb_gazebo_msgs):/cdir/simulator_ws/src/tb_gazebo_msgs \
    -v $(readlink -f simulator_ws/src/tb_gazebo_plugins):/cdir/simulator_ws/src/tb_gazebo_plugins
