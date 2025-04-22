ROS_DISTRIBUTION := humble
PACKAGE_NAME := control
WORKSPACE_ROOT := /project/ros_robotics

.PHONY: all build test clean source run

all: build source

build:
	@echo "Building package..."
	@bash -c "source /opt/ros/$(ROS_DISTRIBUTION)/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          colcon build --packages-select $(PACKAGE_NAME) --symlink-install"

source: build
	@echo "Sourcing environment..."
	@bash -c "source /opt/ros/$(ROS_DISTRIBUTION)/setup.bash && \
	          source install/setup.bash"

run: source
	@echo "Running ROS nodes..."
	@bash -c "source /opt/ros/$(ROS_DISTRIBUTION)/setup.bash && \
	          source install/setup.bash && \
	          ros2 run $(PACKAGE_NAME) observation_publisher & \
	          ros2 run $(PACKAGE_NAME) planner_subsriber"

test: build
	@echo "Running tests..."
	@bash -c "source /opt/ros/$(ROS_DISTRIBUTION)/setup.bash && \
	          source $(WORKSPACE_ROOT)/install/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          python3 -m pytest src/$(PACKAGE_NAME)/test/ -v || \
	          (colcon test-result --all --verbose; false)"

clean:
	@echo "Cleaning workspace..."
	@rm -rf $(WORKSPACE_ROOT)/build $(WORKSPACE_ROOT)/install $(WORKSPACE_ROOT)/log