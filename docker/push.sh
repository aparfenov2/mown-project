_PUSH="push"
[ "$1" == "--pull" ] && {
   _PUSH="pull"
}
set -ex
for tag in mower-core-base mower-core bridge build google mower-gui-base mower-gui mower-sim pytorch core-dev-x86
do
   docker ${_PUSH} kan-rt.ddns.net:8929/docker/ros-mower:$tag
done
