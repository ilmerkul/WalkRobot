DOCKER_IMAGE := ros2-gazebo
DOCKERFILE := ./docker/image/Dockerfile
BUILD_LOG := ./docker/image/build.log

PROJECT_DIR := $(shell pwd)
CONTAINER_WORKDIR := /project

X11_SUPPORT := $(if $(DISPLAY),--env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw",)

.PHONY: install build up up_gui xhost_allow clean isort flake8 check_push

install:
	python3 -m poetry lock
	python3 -m poetry install

build:
	@echo "Building Docker image..."
	sudo docker image build -f $(DOCKERFILE) --tag $(DOCKER_IMAGE) . > $(BUILD_LOG) 2>&1
	@echo "Build complete. Log saved to $(BUILD_LOG)"

up: build
	sudo docker run -it \
					-e CONTAINER_WORKDIR=$(CONTAINER_WORKDIR) \
					--volume="$(PROJECT_DIR):$(CONTAINER_WORKDIR)" \
					$(DOCKER_IMAGE) bash

up_gui: xhost_allow build
	sudo docker run -it \
					-e CONTAINER_WORKDIR=$(CONTAINER_WORKDIR) \
					--volume="$(PROJECT_DIR):$(CONTAINER_WORKDIR)" \
					$(X11_SUPPORT) $(DOCKER_IMAGE) bash

xhost_allow:
	@if [ -z "$$DISPLAY" ]; then \
		echo "X11 display not detected. GUI may not work."; \
	else \
		xhost +local:root; \
		echo "Allowed X11 connections from Docker containers"; \
	fi

clean:
	@echo "Cleaning up Docker containers..."
	@sudo docker ps -aq | xargs -r sudo docker rm -f
	@echo "Cleaning up Docker images..."
	@sudo docker images -q $(DOCKER_IMAGE) | xargs -r sudo docker rmi -f

isort:
	sudo python3 -m isort .

flake8:
	sudo python3 -m flake8

check_push: isort flake8
	python3 -m poetry lock