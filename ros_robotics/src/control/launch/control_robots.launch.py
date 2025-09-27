from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

package_name_dl_control = "dl_control"
package_name_planner = "planner"


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="namespace",
    )

    launch_args = [
        namespace_arg,
    ]

    namespace = LaunchConfiguration("namespace")

    dl_control_robots = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare(package_name_dl_control),
                "launch",
                "dl_control_robots.launch.py",
            ]
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    planner = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(package_name_planner), "launch", "planner.launch.py"]
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    planner_timer = TimerAction(
        period=3.0,
        actions=[planner],
    )

    dl_control_robots_timer = TimerAction(
        period=6.0,
        actions=[dl_control_robots],
    )

    return LaunchDescription(
        [
            *launch_args,
            planner_timer,
            dl_control_robots_timer,
        ]
    )
