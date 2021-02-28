
[ "$1" == "--inner" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    set -e
    cd /cdir/ws && catkin_make
    cd /cdir
    . /cdir/ws/devel/setup.sh
    # roslaunch segmentation_node freiburg.launch
    # roslaunch segmentation_node kitti.launch
    roslaunch segmentation_node kobuki.launch
    exit 0
}

bash _run_in_docker.sh --script $0 --name run_segmentation \
    -v $(readlink -f ws/src/segmentation_node):/cdir/ws/src/segmentation_node \
    -v $(readlink -f ws/src/projecttion_node):/cdir/ws/src/projection_node \
    -v $(readlink -f deeplab/rgbd_dataset_freiburg3_long_office_household.bag):/cdir/ws/src/segmentation_node/bags/rgbd_dataset_freiburg3_long_office_household.bag \
    -v $(readlink -f kitti/kitti.bag):/cdir/ws/src/segmentation_node/bags/kitti.bag
