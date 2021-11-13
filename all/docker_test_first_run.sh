
sudo xhost +local:root

VOLUMES=()
for f in $(find ws/src -type l); do
	ff=${f:7}
	# echo $ff 
    VOLUMES+=("-v $(readlink -f $f):/catkin_ws/src/$ff")
done

docker run \
	-it \
	-p 8088:8088 \
	--name $(cat docker/image) \
	--gpus all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	${VOLUMES[@]} \
    $(cat docker/image) bash

# docker start goofy_mayer