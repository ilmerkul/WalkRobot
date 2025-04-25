from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    observation_publisher_node = Node(
        package="control",
        executable="observation_publisher",
        name="observation_publisher",
        namespace="observation",
    )

    planner_subscriber_node = Node(
        package="control",
        executable="planner_subscriber",
        name="planner_subscriber",
        namespace="planner",
    )

    ld.add_action(observation_publisher_node)
    ld.add_action(planner_subscriber_node)

    return ld
