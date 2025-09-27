from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node

package_name = "dl_control"


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

    namespace = [namespace, TextSubstitution(text="control")]

    stand_up_node = Node(
        package=package_name,
        executable="stand_up_node",
        name="stand_up_node",
        parameters=[
            {"train": True},
        ],
        namespace=namespace,
    )

    stand_node = Node(
        package=package_name,
        executable="stand_node",
        name="stand_node",
        parameters=[
            {"train": True},
        ],
        namespace=namespace,
    )

    return LaunchDescription(
        [
            *launch_args,
            stand_up_node,
            stand_node,
        ]
    )
