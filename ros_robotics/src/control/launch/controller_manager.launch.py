from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
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
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", description="Use sim (gazebo) time", default_value="true"
    )
    control_file_arg = DeclareLaunchArgument(
        name="control_file",
        default_value="control.yaml",
        description="Control file .yaml",
    )
    robot_namespace_arg = DeclareLaunchArgument(
        name="robot_namespace",
        default_value="tropy_spot",
        description="robot namespace",
    )

    launch_args = [
        use_sim_time_arg,
        control_file_arg,
        robot_namespace_arg,
    ]
    control_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            LaunchConfiguration("control_file"),
        ]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")

    robot_control_namespace = PythonExpression(
        ["'/' + '", robot_namespace, f"' + '/{package_name}'"]
    )
    robot_description_namespace = PythonExpression(
        ["'/' + '", robot_namespace, f"' + '/{package_name_description}'"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace=robot_control_namespace,
        parameters=[
            {"use_sim_time": use_sim_time},
            control_file,
        ],
        remappings=[
            (
                "/robot_description",
                PythonExpression(
                    ["'", robot_description_namespace, "' + '/robot_description'"]
                ),
            ),
            (
                "/tf_static",
                PythonExpression(
                    ["'", robot_description_namespace, "' + '/tf_static'"]
                ),
            ),
            (
                "/tf",
                PythonExpression(["'", robot_description_namespace, f"' + '/tf'"]),
            ),
        ],
        output="both",
    )

    effort_control = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare(package_name),
                "launch",
                "effort_control.launch.py",
            ]
        ),
        launch_arguments={
            "robot_namespace": robot_namespace,
        }.items(),
    )

    ros2_control_node_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[effort_control],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            *launch_args,
            ros2_control_node,
            ros2_control_node_handler,
        ]
    )
