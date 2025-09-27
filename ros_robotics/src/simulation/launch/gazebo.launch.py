from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # gz_server_launch = IncludeLaunchDescription(
    #    PathJoinSubstitution(
    #        [FindPackageShare("ros_gz_sim"), "launch", "gz_server.launch.py"]
    #    ),
    #    launch_arguments={
    #        "world_sdf_file": world,
    #        "container_name": container_name,
    #        "create_own_container": "false",
    #        "use_composition": "false",
    #        #bridge
    #        "bridge_name": "ros_gz_bridge",
    #        "config_file": bridge_config,
    #        "container_name": "ros_gz_container",
    #        "create_own_container": "False",
    #        "namespace": "",
    #        "use_composition": "False",
    #        "use_respawn": "False",
    #        "log_level": "info",
    #        "bridge_params": "",
    #    }.items(),
    # )

    gz_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
            "gui": "false",
        }.items(),
    )

    ros_gz_bridge_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("ros_gz_bridge"), "launch", "clock_bridge.launch"]
        ),
        launch_arguments={
            "bridge_name": "ros_gz_bridge",
        }.items(),
    )

    # ros_gz_bridge_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        PathJoinSubstitution(
    #            [FindPackageShare("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py"]
    #        )
    #    ),
    #    launch_arguments={
    #        "bridge_name": "ros_gz_bridge",
    #        "config_file": bridge_config,
    #        "container_name": "ros_gz_container",
    #        "create_own_container": "False",
    #        "namespace": "",
    #        "use_composition": "False",
    #        "use_respawn": "False",
    #        "log_level": "info",
    #        "bridge_params": '',
    #    }.items(),
    # )

    world_control_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_control_bridge",
        arguments=[
            "/world/world_demo/control@ros_gz_interfaces/srv/ControlWorld",
            "/world/world_demo/remove@ros_gz_interfaces/srv/DeleteEntity",
        ],
        output="both",
    )

    bridge_timer = TimerAction(
        period=10.0, actions=[ros_gz_bridge_launch, world_control_bridge]
    )

    check_gazebo = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "until ros2 topic list | grep -q '/clock'; do sleep 2; echo 'Waiting for Gazebo...'; done",
        ],
        output="both",
        shell=False,
    )

    control_sim = Node(
        package=package_name,
        executable="control_sim",
        name="control_sim",
        output="both",
    )

    handler_check_gazebo = RegisterEventHandler(
        OnProcessExit(
            target_action=check_gazebo,
            on_exit=[
                TimerAction(
                    period=10.0,
                    actions=[control_sim],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            *launch_args,
            gz_server_launch,
            bridge_timer,
            check_gazebo,
            handler_check_gazebo,
        ]
    )
