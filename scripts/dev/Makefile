include .env


build:
	./dev_build.sh

build_and_run_test:
	./dev_build.sh

start:
	docker start ${CONTAINER_NAME}

join:
	docker exec -ti ${CONTAINER_NAME} bash

stop:
	docker stop ${CONTAINER_NAME}

rm:
	docker stop ${CONTAINER_NAME}
