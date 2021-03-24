
A_LOAM_DIR="ws/src/A-LOAM"

pushd $PWD
cd ${A_LOAM_DIR}/docker
make build
popd

docker run \
-it \
--rm \
--name loam \
--net=host \
-v ${A_LOAM_DIR}:/root/catkin_ws/src/A-LOAM/ \
ros:aloam-noetic \
/bin/bash -c \
"cd /root/catkin_ws/; \
    source devel/setup.bash; \
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch rviz:=false"
