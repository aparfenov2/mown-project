ALL="$@"
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --inner) INNER=1 ;;
        --bypass) BYPASS="bypass:=true" ;;
        *) echo unrocognized $1; exit 1 ;;
    esac
    shift
done

[ -n "$INNER" ] && {
    [ -n "$BYPASS" ] && {
        echo WARNING: segmentation disabled
    }
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    set -e
    cd /cdir/ws && catkin_make
    cd /cdir
    . /cdir/ws/devel/setup.sh
    # roslaunch segmentation_node freiburg.launch
    # roslaunch segmentation_node kitti.launch    
    roslaunch segmentation_node kobuki.launch ${BYPASS}
    exit 0
}

PTH="ws/src/segmentation_node/model/fcn_hr18s_512x1024_40k_cityscapes_20200601_014216-93db27d0.pth"
[ -f "$PTH" ] || {
    curl -o $PTH  https://download.openmmlab.com/mmsegmentation/v0.5/hrnet/fcn_hr18s_512x1024_40k_cityscapes/fcn_hr18s_512x1024_40k_cityscapes_20200601_014216-93db27d0.pth
}
[ -d "ws/src/catkin_simple" ] || {
    pushd $PWD
    cd ws/src/
    git clone git@github.com:catkin/catkin_simple.git
    popd
}

bash _run_in_docker.sh --script $0 --name run_segmentation \
    -v $(readlink -f ws):/cdir/ws \
    -v $(readlink -f ws/src/segmentation_node):/cdir/ws/src/segmentation_node \
    -v $(readlink -f deeplab/rgbd_dataset_freiburg3_long_office_household.bag):/cdir/ws/src/segmentation_node/bags/rgbd_dataset_freiburg3_long_office_household.bag \
    -v $(readlink -f kitti/kitti.bag):/cdir/ws/src/segmentation_node/bags/kitti.bag \
    $ALL
