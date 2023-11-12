include .env

build:
	docker build -t ${IMAGE_NAME} -f studio/Dockerfile studio/

run:
	docker run -d -it -p ${PORT}:8080 --name ${CONTAINER_NAME} ${IMAGE_NAME}

start:
	docker start ${CONTAINER_NAME}

join:
	docker exec -ti ${CONTAINER_NAME} bash

stop:
	docker stop ${CONTAINER_NAME}

rm: stop
	docker rm ${CONTAINER_NAME}
