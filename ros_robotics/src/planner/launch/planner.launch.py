from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

package_name = "planner"


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

    namespace = [namespace, TextSubstitution(text="planner")]

    planner_node = Node(
        package=package_name,
        executable="planner_node",
        name="planner_node",
        namespace=namespace,
    )

    return LaunchDescription(
        [
            *launch_args,
            planner_node,
        ]
    )
