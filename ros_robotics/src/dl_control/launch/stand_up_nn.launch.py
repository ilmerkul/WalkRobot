from launch import LaunchDescription
from launch_ros.actions import Node

package_name = "dl_control"


def generate_launch_description():
    stand_up_nn_node = Node(
        package=package_name,
        executable="stand_up_nn_node",
        name="stand_up_nn",
        namespace="control",
    )

    return LaunchDescription(
        [
            stand_up_nn_node,
        ]
    )
