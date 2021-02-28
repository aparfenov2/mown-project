
set -ex

[ "$1" == "--inner" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    . /cdir/simulator_ws/devel/setup.sh
    rosrun xacro xacro '/cdir/simulator_ws/src/tb_gazebo_description/robots/kobuki_hexagons_kinect.urdf.xacro' > robot_description
    # check_urdf robot_description
    exit 0
}

bash _run_in_docker.sh --script $0
