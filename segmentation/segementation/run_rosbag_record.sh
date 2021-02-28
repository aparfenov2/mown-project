[ -z "$1" ] && {
    echo specify bag file
    exit 1
}

[ "$1" == "--inner" ] && {
    shift
    . "/opt/ros/$ROS_DISTRO/setup.bash"
set -ex
    rosbag record -o $1 \
        rosout \
        rosout_agg \
        tf \
        tf_static \
        /mobile_base/commands/velocity \
        /camera/depth/camera_info \
        /camera/depth/image_raw \
        /camera/depth/points \
        /camera/parameter_descriptions \
        /camera/parameter_updates \
        /clock \
        /camera/rgb/image_raw/compressed \
        /velodyne_points \
        /odom \
        __name:=my_bag
    rosnode kill /my_bag
    exit 0
}

bash _run_in_docker.sh --script $0 --name rosbag_record $@
