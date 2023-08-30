[ -z "$1" ] && {
    echo specify bag file
    exit 1
}

[ "$1" == "--inner" ] && {
    shift
    . "/opt/ros/$ROS_DISTRO/setup.bash"
set -ex
    rosbag record -o $1 \
        tf \
        tf_static \
        /segmentation_node/segmentation_vis \
        /pcl_proc/points2 \
        /clock \
        __name:=my_bag
    rosnode kill /my_bag
    exit 0
}

bash _run_in_docker.sh --script $0 --name rosbag_record $@
