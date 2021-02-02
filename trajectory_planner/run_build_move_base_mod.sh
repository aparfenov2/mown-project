
set -ex
ROOT_DIR="$(dirname $PWD/$0)"
echo ROOT_DIR=${ROOT_DIR}

export TURTLEBOT3_MODEL=burger

[ "$1" == "--inner" ] && {
    . "/opt/ros/$ROS_DISTRO/setup.bash"
    . /cdir/catkin_ws/devel/setup.sh
    cd /cdir/catkin_ws && catkin_make
    exit 0
}

IMAGE=ros-mower
NAME=mower-sim
SCRIPT=$0

bash _run_in_docker.sh --script /cdir/$0 --name run_sim
