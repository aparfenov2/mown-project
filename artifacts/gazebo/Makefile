include .env


build:
	./build.sh

run_dev: rm
	./run.sh

run_prod:
	./run.sh

start:
	xhost +local:root
	docker start ${CONTAINER_NAME}

join:
	xhost +local:root
	docker exec -ti ${CONTAINER_NAME} bash

stop:
	docker stop ${CONTAINER_NAME}

rm: stop
	docker rm ${CONTAINER_NAME}
