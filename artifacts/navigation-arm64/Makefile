include .env


build:
	./build.sh

run_dev:
	./run.sh

run_prod:
	./run.sh

start:
	docker start ${CONTAINER_NAME}

join:
	docker exec -ti ${CONTAINER_NAME} bash

stop:
	docker stop ${CONTAINER_NAME}

rm: stop
	docker rm ${CONTAINER_NAME}
