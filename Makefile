include .env
export

.PHONY: install build up up_gui xhost_allow clean

build:
	@echo "Building Docker image..."
	@./docker/build.sh
	@echo "Build complete. Log saved to $(BUILD_LOG)"

up:
	@echo "Running Docker image..."
	@./docker/run.sh

up_gui: xhost_allow
	@echo "Running Docker image..."
	@./docker/run_gui.sh

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

install:
	@echo "Install packages"
	@./docker/install.sh $(PROJECT_DIR)/.venv