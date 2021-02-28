
[ "$1" == "--inner" ] && {
    shift
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    set -ex
    roscore &
    rosbag play --loop $@
    exit 0
}

bash _run_in_docker.sh --script $0 --name rosbag_play $@
