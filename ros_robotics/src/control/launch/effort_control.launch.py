from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "control"
package_name_description = "description"
package_name_dl_control = "dl_control"
package_name_planner = "planner"


def generate_launch_description():
    robot_namespace_arg = DeclareLaunchArgument(
        name="robot_namespace",
        default_value="tropy_spot",
        description="robot namespace",
    )
    control_file_arg = DeclareLaunchArgument(
        name="control_file",
        default_value="control.yaml",
        description="Control file .yaml",
    )

    launch_args = [
        robot_namespace_arg,
        control_file_arg,
    ]

    control_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            LaunchConfiguration("control_file"),
        ]
    )

    robot_namespace = LaunchConfiguration("robot_namespace")

    robot_control_namespace = PythonExpression(
        ["'/' + '", robot_namespace, f"' + '/{package_name}'"]
    )
    robot_controller_manager_namespace = PythonExpression(
        ["'/' + '", robot_namespace, f"' + '/{package_name}/controller_manager'"]
    )
    robot_description_namespace = PythonExpression(
        ["'/' + '", robot_namespace, f"' + '/{package_name_description}'"]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        namespace=robot_control_namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            robot_controller_manager_namespace,
            "--switch-timeout",
            "10",
            "--ros-args",
            "--remap",
            PythonExpression(
                [
                    "'",
                    robot_control_namespace,
                    "' + '/joint_states:=' + '",
                    robot_description_namespace,
                    "' + '/joint_states'",
                ]
            ),
            "--remap",
            PythonExpression(
                [
                    "'",
                    robot_control_namespace,
                    "' + '/dynamic_joint_states:=' + '",
                    robot_description_namespace,
                    "' + '/dynamic_joint_states'",
                ]
            ),
        ],
        parameters=[control_file],
    )

    robot_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="robot_effort_controller_spawner",
        namespace=robot_control_namespace,
        arguments=[
            "effort_controller",
            "--controller-manager",
            robot_controller_manager_namespace,
            "--switch-timeout",
            "10",
        ],
        parameters=[control_file],
    )

    joint_state_broadcaster_spawner_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[robot_effort_controller_spawner],
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            *launch_args,
            joint_state_broadcaster_spawner,
            robot_effort_controller_spawner,
        ]
    )
