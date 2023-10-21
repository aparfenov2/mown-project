# docker run -it \
#     --user=$(id -u $USER):$(id -g $USER) \
#     --env="DISPLAY" \
#     --volume="/etc/group:/etc/group:ro" \
#     --volume="/etc/passwd:/etc/passwd:ro" \
#     --volume="/etc/shadow:/etc/shadow:ro" \
#     --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     -v `pwd`:/catkin_ws/src \
#     --name sleepy_mendeleev \
#     enginx_image bash
#     # -v `pwd`:/catkin_ws/src \
#     # great_gates
#     # ros-mown bash
#     # # sleepy_mendeleev

name='sleepy_mendeleev'
sudo xhost +local:root

docker run \
	--rm \
	-it \
	--gpus all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
    -v `pwd`:/catkin_ws/src \
    --name $name \
    ros-mown bash