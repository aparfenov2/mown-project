set -ex
for tag in mower-core-base mower-core bridge build google mower-gui-base mower-gui mower-sim pytorch
do
   docker build -f Dockerfile:$tag -t kan-rt.ddns.net:8929/docker/ros-mower:$tag .
done

docker build -f core-dev-x86/Dockerfile -t kan-rt.ddns.net:8929/docker/ros-mower:core-dev-x86 core-dev-x86
