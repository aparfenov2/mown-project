ALL="$@"
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --inner) INNER=1 ;;
        --teleop) TELEOP=1 ;;
        *) echo unrocognized $1; exit 1 ;;
    esac
    shift
done

[ -n "$INNER" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    set -ex
    pushd $PWD
    # cd ws && catkin_make
    popd
    . ws/devel/setup.sh
    [ -z "$TELEOP" ] && {
        # roslaunch elevation_mapping_demos turtlesim3_waffle_demo.launch
        # roslaunch projection_node segmentation_input.launch
        # rosservice call -- /elevation_mapping/get_submap odom -0.5 0.0 0.5 1.2 []
        # rosservice call /elevation_mapping/save_map "file_path: '/cdir/elevation_map.bag'"
        # rostopic echo /elevation_mapping/elevation_map
    }

    [ -n "$TELEOP" ] && {
        export TURTLEBOT3_MODEL=waffle
        roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    }
    exit 0
}

bash _run_in_docker.sh --script $0 \
    -v $(readlink -f ws/src/projection_node):/cdir/ws/src/projection_node \
    $ALL
