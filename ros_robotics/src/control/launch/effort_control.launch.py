from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "control"
package_name_description = "description"
package_name_dl_control = "dl_control"
package_name_planner = "planner"


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        namespace="/tropy_spot_0/control",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/tropy_spot_0/control/controller_manager",
            "--switch-timeout",
            "10",
        ],
    )

    robot_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="robot_effort_controller_spawner",
        namespace="/tropy_spot_0/control",
        arguments=[
            "effort_controller",
            "--controller-manager",
            "/tropy_spot_0/control/controller_manager",
            "--switch-timeout",
            "10",
        ],
    )

    delay_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                robot_effort_controller_spawner,
            ],
        )
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            delay_after_joint_state_broadcaster_spawner,
        ]
    )
