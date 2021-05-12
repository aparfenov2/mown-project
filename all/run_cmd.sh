set -e

[ "$1" == "--inner" ] && {
    shift
    . "/opt/ros/$ROS_DISTRO/setup.bash"    
    . /cdir/ws/devel/setup.bash
    $@
    exit 0
}


VOLUMES=()
for f in $(find ws/src -type l); do
    VOLUMES+=("-v $(readlink -f $f):/cdir/$f")
done
bash _run_in_docker.sh --script $0 \
    ${VOLUMES[@]} \
    $@

# docker run -ti --rm --network=host -v $PWD:/cdir ros-mower $@
