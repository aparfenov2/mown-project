include .env


build:
	./build.sh

run:
	./run.sh

start:
	docker start ${CONTAINER_NAME}

join:
	docker exec -ti ${CONTAINER_NAME} bash

stop:
	docker stop ${CONTAINER_NAME}

rm: stop
	docker rm ${CONTAINER_NAME}
