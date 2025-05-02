from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "control"
package_name_description = "description"


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

    launch_args = [use_sim_time_arg, control_file_arg, xacro_file_arg]
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
        parameters=[robot_description_content, control_file],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--switch-timeout",
            "10",
        ],
    )

    robot_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            control_file,
            "--switch-timeout",
            "10",
        ],
    )

    control_node = Node(
        package=package_name,
        executable="control_node",
        name="control_node",
        namespace="control_node",
    )

    # robot_controller_gazebo = Node(
    #    package="bruno_controller",
    #    executable="robot_controller_gazebo",
    #    name="robot_controller_gazebo",
    #    output="screen",
    #    parameters=[],
    # )

    # joypad_node = Node(
    #    package="joypad",
    #    executable="ramped_joypad.py",
    #    name="ramped_joypad",
    #    output="screen",
    #    parameters=[],
    # )

    delay_robot_postion_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_position_controller_spawner],
            )
        )
    )

    delay_control_node_after_robot_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_position_controller_spawner,
            on_exit=[control_node],
        )
    )

    return LaunchDescription(
        [
            *launch_args,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            delay_robot_postion_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_control_node_after_robot_position_controller_spawner,
        ]
    )
