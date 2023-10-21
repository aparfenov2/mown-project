
[ "$1" == "--inner" ] && {
    shift
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    set -ex
    roscore &
    sleep 1
    rosparam set use_sim_time true
    rosbag play --clock --loop $1
    exit 0
}

bash _run_in_docker.sh --script $0 --name rosbag_play -v $(readlink -f $1):/cdir/$(basename $1) /cdir/$(basename $1)
