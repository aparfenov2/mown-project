include .env

build:
	docker build -t ${IMAGE_NAME} \
		-f ../../docker/rosbridge/Dockerfile ${DOCKER_PATH} .

run:
	docker run \
		--rm \
		-it \
		-v "${PWD}/../../modules/msgs:/catkin_ws/src/" \
		--net=host \
		--name ${CONTAINER_NAME} \
		${IMAGE_NAME} bash run_inside.sh 

start:
	docker start ${CONTAINER_NAME}

join:
	docker exec -ti ${CONTAINER_NAME} bash

stop:
	docker stop ${CONTAINER_NAME}

rm: stop
	docker rm ${CONTAINER_NAME}
