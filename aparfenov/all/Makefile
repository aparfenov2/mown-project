UP=up $(DAEMON) --no-recreate
DOWN=down --remove-orphans

.PHONY: gen_dc

gen_dc:
	bash gen_docker_compose.sh

up: gen_dc
	docker compose $(UP) $(ARGS)

down: gen_dc
	docker compose $(DOWN)

all: gen_dc
	ROSARGS="world:=baylands" docker compose --profile backend_demo --profile costmap_demo $(UP)

backend_demo: gen_dc
	docker compose --profile backend_demo $(UP)

build_images: gen_dc
	docker compose --profile all build

pull: gen_dc
	docker compose --profile all pull

push: gen_dc
	docker compose --profile all push

# docker-compose individual services
build: gen_dc
	docker compose run build

sim: gen_dc
	docker compose $(UP) $@

rviz: gen_dc
	docker compose $(UP) $@

rqt: gen_dc
	docker compose $(UP) $@

backend: gen_dc
	docker compose $(UP) $@

planning: gen_dc
	docker compose $(UP) $@

costmap_demo: gen_dc
	docker compose $(UP) $@

bridge: gen_dc
	docker compose $(UP) $@

foxglove: gen_dc
	docker compose $(UP) $@

ps: gen_dc
	docker compose $@
