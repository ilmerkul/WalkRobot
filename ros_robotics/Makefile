include ../.env
export

.PHONY: all build_all source build_control build_simulation build_description run_control run_simulation run_description_display test_control test_simulation test_description clean source

all: build_all source

build_all: build_control build_simulation build_description

test_all: test_control test_simulation test_description

source:
	@echo "Sourcing environment..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	          source $(WORKSPACE_ROOT)/install/setup.bash"

build_control:
	@echo "Building control package..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          colcon build --packages-select $(PACKAGE_CONTROL_NAME) --symlink-install"

build_simulation:
	@echo "Building simulation package..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          colcon build --packages-select $(PACKAGE_SIMULATION_NAME) --symlink-install"

build_description:
	@echo "Building description package..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          colcon build --packages-select $(PACKAGE_DESCRIPTION_NAME) --symlink-install"

run_control: build_control
	@echo "Running ROS control nodes..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \Waiting
	          ros2 launch $(PACKAGE_CONTROL_NAME) control.launch.py"

run_simulation: build_simulation build_control build_description
	@echo "Running ROS simulation nodes..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          ros2 launch $(PACKAGE_SIMULATION_NAME) gazebo.launch.py"

run_description_display: build_description
	@echo "Running ROS description nodes..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          ros2 launch $(PACKAGE_DESCRIPTION_NAME) display.launch.py"

test_control: build_control
	@echo "Running tests..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          python3 -m pytest src/$(PACKAGE_CONTROL_NAME)/test/ -v || \
	          (colcon test-result --all --verbose; false)"

test_simulation: build_simulation
	@echo "Running tests..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          python3 -m pytest src/$(PACKAGE_SIMULATION_NAME)/test/ -v || \
	          (colcon test-result --all --verbose; false)"

test_description: build_simulation
	@echo "Running tests..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          python3 -m pytest src/$(PACKAGE_DESCRIPTION_NAME)/test/ -v || \
	          (colcon test-result --all --verbose; false)"

clean:
	@echo "Cleaning workspace..."
	@echo $(WORKSPACE_ROOT)
	@rm -rf $(WORKSPACE_ROOT)/build $(WORKSPACE_ROOT)/install $(WORKSPACE_ROOT)/log ./core
