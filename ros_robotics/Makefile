ROS_DISTRIBUTION := humble

WORKSPACE_ROOT := /project/ros_robotics

PACKAGE_CONTROL_NAME := control
PACKAGE_SIMULATION_NAME := simulation

.PHONY: all build test clean source run

all: build_all source

build_all: build_control build_simulation

source:
	@echo "Sourcing environment..."
	@bash -c "source /opt/ros/$(ROS_DISTRIBUTION)/setup.bash && \
	          source $(WORKSPACE_ROOT)/install/setup.bash"

build_control:
	@echo "Building control package..."
	@bash -c "source /opt/ros/$(ROS_DISTRIBUTION)/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          colcon build --packages-select $(PACKAGE_CONTROL_NAME) --symlink-install"

build_simulation:
	@echo "Building simulation package..."
	@bash -c "source /opt/ros/$(ROS_DISTRIBUTION)/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          colcon build --packages-select $(PACKAGE_SIMULATION_NAME) --symlink-install"

run_control: build_control
	@echo "Running ROS control nodes..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          ros2 launch $(PACKAGE_CONTROL_NAME) control.launch.py"

run_simulation: build_simulation
	@echo "Running ROS simulation nodes..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          ros2 launch $(PACKAGE_SIMULATION_NAME) display.launch.py"

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

clean:
	@echo "Cleaning workspace..."
	@rm -rf $(WORKSPACE_ROOT)/build $(WORKSPACE_ROOT)/install $(WORKSPACE_ROOT)/log