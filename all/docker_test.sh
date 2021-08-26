
sudo xhost +local:root

VOLUMES=()
for f in $(find ws/src -type l); do
	ff=${f:7}
	# echo $ff 
    VOLUMES+=("-v $(readlink -f $f):/catkin_ws/src/$ff")
done

# docker run \
# 	-it \
# 	--gpus all \
# 	-v /tmp/.X11-unix:/tmp/.X11-unix \
# 	-e DISPLAY=$DISPLAY \
# 	-e QT_X11_NO_MITSHM=1 \
# 	${VOLUMES[@]} \
#     ros-mower bash

docker start goofy_mayer