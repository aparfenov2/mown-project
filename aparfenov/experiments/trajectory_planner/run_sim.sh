
set -ex
ROOT_DIR="$(dirname $PWD/$0)"
echo ROOT_DIR=${ROOT_DIR}

export TURTLEBOT3_MODEL=waffle

[ -d "catkin_ws/src" ] || {
    mkdir -p catkin_ws/src || true
    cd catkin_ws/src
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
}

[ "$1" == "--inner" ] && {
    # . /cdir/init_intel.sh
    # ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    # . /cdir/init_kinetic.sh
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    # cd /cdir/catkin_ws catkin_make
    . /cdir/catkin_ws/devel/setup.sh
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    exit 0
}

IMAGE=ros-mower
NAME=mower-sim
SCRIPT=$0

bash _run_in_docker.sh --script $0 --name run_sim
