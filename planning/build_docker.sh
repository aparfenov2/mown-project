docker build -t  ros-mown:latest .

name='sleepy_mendeleev'

if [ ! "$(docker ps -q -f name=$name)" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=$name)" ]; then
        # cleanup
        docker rm $name
    fi
    # # run your container
    # docker run -d --name <name> my-docker-image
fi

# XSOCK=/tmp/.X11-unix
# XAUTH=/tmp/.docker.xauth
# touch $XAUTH
# xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# docker run -it \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v `pwd`:/catkin_ws/src \
#     --name sleepy_mendeleev \
#     ros-mown bash
    # --env="DISPLAY=$DISPLAY" \
    # --env="QT_X11_NO_MITSHM=1" \
    # --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    # --env="XAUTHORITY=$XAUTH" \
    # --volume="$XAUTH:$XAUTH" \
    # --net=host \
    # --privileged \
    # -v `pwd`:/catkin_ws/src \
    # --name sleepy_mendeleev \
    # ros-mown bash


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
