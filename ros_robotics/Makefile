include ../.env
export

.PHONY: all build_interface build_description build_dl_control build_planner build_control build_simulation run_control run_simulation run_description_display test_control test_simulation test_description test_dl_control test_planner clean

build_all: build_interface build_description build_dl_control build_planner build_control build_simulation

test_all: test_control test_simulation test_description test_dl_control test_planner

build_interface:
	@echo "Building interface package..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
			  colcon build --packages-select $(PACKAGE_INTERFACE_NAME) --symlink-install"

build_control:
	@echo "Building control package..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	          colcon build --packages-select $(PACKAGE_CONTROL_NAME) --symlink-install"

build_dl_control:
	@echo "Building dl_control package..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	          colcon build --packages-select $(PACKAGE_DL_CONTROL_NAME) --symlink-install"

build_simulation:
	@echo "Building simulation package..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	          colcon build --packages-select $(PACKAGE_SIMULATION_NAME) --symlink-install"

build_description:
	@echo "Building description package..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	          colcon build --packages-select $(PACKAGE_DESCRIPTION_NAME) --symlink-install"

build_planner:
	@echo "Building description package..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	          colcon build --packages-select $(PACKAGE_PLANNER_NAME) --symlink-install"

run_simulation: build_all
	@echo "Running ROS simulation nodes..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && \
			  source $(WORKSPACE_ROOT)/install/setup.bash && \
	          ros2 launch $(PACKAGE_SIMULATION_NAME) gazebo.launch.py"

run_description_display: build_description build_control
	@echo "Running ROS description nodes..."
	@bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash \
			  source $(WORKSPACE_ROOT)/install/setup.bash && \
	          ros2 launch $(PACKAGE_DESCRIPTION_NAME) display.launch.py"

test_control:
	@echo "Running tests..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          python3 -m pytest src/$(PACKAGE_CONTROL_NAME)/test/ -v || \
	          (colcon test-result --all --verbose; false)"

test_simulation:
	@echo "Running tests..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          python3 -m pytest src/$(PACKAGE_SIMULATION_NAME)/test/ -v || \
	          (colcon test-result --all --verbose; false)"

test_description:
	@echo "Running tests..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          python3 -m pytest src/$(PACKAGE_DESCRIPTION_NAME)/test/ -v || \
	          (colcon test-result --all --verbose; false)"

test_dl_control:
	@echo "Running tests..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          python3 -m pytest src/$(PACKAGE_DL_CONTROL_NAME)/test/ -v || \
	          (colcon test-result --all --verbose; false)"

test_planner:
	@echo "Running tests..."
	@bash -c "source $(WORKSPACE_ROOT)/install/setup.bash && \
	          cd $(WORKSPACE_ROOT) && \
	          python3 -m pytest src/$(PACKAGE_PLANNER_NAME)/test/ -v || \
	          (colcon test-result --all --verbose; false)"

clean:
	@echo "Cleaning workspace..."
	@echo $(WORKSPACE_ROOT)
	@rm -rf $(WORKSPACE_ROOT)/build $(WORKSPACE_ROOT)/install $(WORKSPACE_ROOT)/log ./core
