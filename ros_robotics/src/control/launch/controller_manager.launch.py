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
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", description="Use sim (gazebo) time", default_value="true"
    )
    control_file_arg = DeclareLaunchArgument(
        name="control_file",
        default_value="robot.yaml",
        description="Control file .yaml",
    )
    xacro_file_arg = DeclareLaunchArgument(
        name="xacro_file",
        description="Name of xacro file (if use_urdf=false)",
    )

    launch_args = [
        use_sim_time_arg,
        control_file_arg,
        xacro_file_arg,
    ]
    control_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            LaunchConfiguration("control_file"),
        ]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name_description),
            "urdf",
            LaunchConfiguration("xacro_file"),
        ]
    )

    robot_description_content = Command(
        ["xacro ", xacro_file, " use_sim_time:=", use_sim_time]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace="/tropy_spot_0/control",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
            {"update_rate": 100},
            control_file,
        ],
        remappings=[
            ("~/robot_description", "/tropy_spot_0/observation/robot_description"),
            ("/tf_static", "/tropy_spot_0/observation/tf_static"),
            ("/tf", "/tropy_spot_0/observation/tf"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        namespace="/tropy_spot_0/control",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/tropy_spot_0/controller_manager",
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
            "/tropy_spot_0/controller_manager",
            "--switch-timeout",
            "10",
        ],
    )

    delay_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
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
            *launch_args,
            ros2_control_node,
            delay_after_ros2_control_node,
            delay_after_joint_state_broadcaster_spawner,
        ]
    )
