UP=up $(DAEMON) --no-recreate

.PHONY: gen_dc

gen_dc:
	bash gen_docker_compose.sh

up: gen_dc
	docker compose $(UP) $(ARGS)

down: gen_dc
	docker compose --profile all $@ --remove-orphans

all: gen_dc download_segmodel
	ROSARGS="world:=baylands" docker compose --profile backend_demo --profile costmap_demo $(UP)

backend_demo: gen_dc
	docker compose --profile $@ $(UP)

build_images: gen_dc
	docker compose --profile all build

pull: gen_dc
	docker compose --profile all $@

push: gen_dc
	docker compose --profile all $@

# docker-compose individual services
planning_build: gen_dc
	docker compose run $@

clean:
	rm -r ws/.catkin_tools ws/build ws/devel ws/pycache || true
	rm -r ws_planning/.catkin_tools ws_planning/build ws_planning/devel ws_planning/pycache || true

build: planning_build
	docker compose run $@

download_segmodel: data/PIDNet_S_ImageNet.pth.tar data/best.pt
data/PIDNet_S_ImageNet.pth.tar:
	mkdir data || true
	cd data && wget http://kan-rt.ddns.net:18000/incoming/pidnet/PIDNet_S_ImageNet.pth.tar

data/best.pt:
	mkdir data || true
	cd data && wget http://kan-rt.ddns.net:18000/incoming/pidnet/best.pt

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

costmap_demo: gen_dc download_segmodel
	docker compose $(UP) $@

bridge: gen_dc
	docker compose $(UP) $@

foxglove: gen_dc
	docker compose $(UP) $@

ps: gen_dc
	docker compose $@
