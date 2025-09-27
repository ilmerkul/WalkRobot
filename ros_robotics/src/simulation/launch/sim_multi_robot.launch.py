from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "simulation"


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        name="gui", description="Use gui gazebo", default_value="true"
    )
    world_arg = DeclareLaunchArgument(
        name="world",
        description="The path to the description of world",
        default_value="empty.world",
    )
    container_name_arg = DeclareLaunchArgument(
        "container_name",
        default_value="ros_gz_container",
        description="Name of container that nodes will load in if use composition",
    )
    bridge_config_arg = DeclareLaunchArgument(
        name="bridge_config",
        description="The path to the config of gz ros bridge",
        default_value="ros_gz_bridge.yaml",
    )

    launch_args = [gui_arg, world_arg, container_name_arg, bridge_config_arg]

    gui = LaunchConfiguration("gui")
    world = PathJoinSubstitution(
        [FindPackageShare(package_name), "worlds", LaunchConfiguration("world")]
    )
    container_name = LaunchConfiguration("container_name")
    bridge_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", LaunchConfiguration("bridge_config")]
    )

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(package_name), "launch", "gazebo.launch.py"]
        ),
        launch_arguments={
            "gui": gui,
            "world": world,
            "container_name": container_name,
            "bridge_config": bridge_config,
        }.items(),
    )

    robots = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(package_name), "launch", "robots.launch.py"]
        ),
        launch_arguments={
            "gui": gui,
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_args,
            gazebo,
            TimerAction(
                period=15.0,
                actions=[robots],
            ),
        ]
    )
