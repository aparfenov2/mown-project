
set -ex

[ "$1" == "--inner" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    set -ex
    pushd $PWD
    cd /cdir/simulator_ws && catkin_make
    popd
    . /cdir/simulator_ws/devel/setup.sh
    ls -l /cdir/simulator_ws/src/benewake_ce30c/
    rosrun xacro xacro '/cdir/simulator_ws/src/tb_gazebo_description/robots/kobuki_hexagons_kinect.urdf.xacro' > robot_description
    # roslaunch benewake_ce30c benewake_ce30c.launch

    # check_urdf robot_description
    exit 0
}

bash _run_in_docker.sh \
    --volume $(readlink -f simulator_ws/src/benewake_ce30c):/cdir/simulator_ws/src/benewake_ce30c \
    --script $0

