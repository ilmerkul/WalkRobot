from launch import LaunchDescription
from launch_ros.actions import Node

package_name = "planner"


def generate_launch_description():
    planner_node = Node(
        package=package_name,
        executable="planner_node",
        name="planner_node",
        namespace="tropy_spot_0",
    )

    return LaunchDescription(
        [
            planner_node,
        ]
    )
