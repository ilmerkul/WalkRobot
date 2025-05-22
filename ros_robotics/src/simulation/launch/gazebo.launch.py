from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    launch_args = [gui_arg, world_arg]

    gui = LaunchConfiguration("gui")
    world = PathJoinSubstitution(
        [FindPackageShare(package_name), "worlds", LaunchConfiguration("world")]
    )

    gzserver_cmd_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"]
        ),
        launch_arguments={
            "world": world,
            "pause": "false",
            "physics": "ode",
            "server_required": "true",
            "gui_required": gui,
        }.items(),
    )

    gzclient_cmd_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"]
        )
    )

    # gazebo = IncludeLaunchDescription(
    #            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", 'gazebo.launch.py']),
    #         )

    check_gazebo = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "until ros2 service list | grep -q '/spawn_entity'; do sleep 1; echo 'Waiting for Gazebo...'; done",
        ],
        output="screen",
        shell=False,  # Important: set to False when using list-form cmd
    )

    spawn_robot_world = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("simulation"), "launch", "spawn_robot.launch.py"]
        ),
        launch_arguments={"gui": gui}.items(),
    )

    return LaunchDescription(
        [
            *launch_args,
            # gazebo,
            gzserver_cmd_launch,
            gzclient_cmd_launch,
            check_gazebo,
            TimerAction(period=2.0, actions=[spawn_robot_world]),
        ]
    )
