set -e

[ "$1" == "--inner" ] && {
    shift
    . "/opt/ros/$ROS_DISTRO/setup.bash"    
    $@
    exit 0
}

bash _run_in_docker.sh --script $0 $@

# docker run -ti --rm --network=host -v $PWD:/cdir ros-mower $@
