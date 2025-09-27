from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "description"


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(name="gui", description="Use gui gazebo")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", description="Use sim (gazebo) time"
    )
    tfconfig_arg = DeclareLaunchArgument(
        name="tfconfig",
        description="Absolute path to tf config file",
        default_value="static_transform_publisher.yaml",
    )
    xacro_file_arg = DeclareLaunchArgument(
        name="xacro_file",
        default_value="robot.urdf.xacro",
        description="Name of xacro file (if use_urdf=false)",
    )
    robot_namespace_arg = DeclareLaunchArgument(
        name="robot_namespace",
        description="robot namespace",
    )

    launch_args = [
        robot_namespace_arg,
        gui_arg,
        use_sim_time_arg,
        tfconfig_arg,
        xacro_file_arg,
    ]

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "xacro",
            "urdf",
            LaunchConfiguration("xacro_file"),
        ]
    )
    tfconfig = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            LaunchConfiguration("tfconfig"),
        ]
    )
    robot_namespace = LaunchConfiguration("robot_namespace")

    robot_description_content = Command(
        [
            "xacro ",
            xacro_file,
            " namespace:=/",
            robot_namespace,
            " exclude_gazebo_tags:=false",
            " exclude_ros2_control:=false",
        ]
    )

    robot_package_namespace = PathJoinSubstitution(["", robot_namespace, package_name])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_package_namespace,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_content,
            }
        ],
        remappings=[
            ("/tf_static", "tf_static"),
            ("/tf", "tf"),
            ("robot_description", "robot_description"),
            ("joint_states", "joint_states"),
        ],
        output="both",
    )

    joint_state_publisher_node_condition = PythonExpression(
        [
            '"joint_state_publisher_gui" if "',
            gui,
            '" in ("true", "True", "1") else "joint_state_publisher"',
        ]
    )

    joint_state_publisher_node = Node(
        package=joint_state_publisher_node_condition,
        executable=joint_state_publisher_node_condition,
        name="joint_state_publisher",
        namespace=robot_package_namespace,
        output="both",
        remappings=[
            ("joint_states", "joint_states"),
        ],
    )

    tf = Node(
        package=package_name,
        executable="tf_wrapper",
        name="static_transform_publisher",
        namespace=robot_package_namespace,
        output="both",
        parameters=[
            {
                "config_file": tfconfig,
                "namespace": robot_package_namespace,
            }
        ],
        remappings=[
            ("/tf_static", "tf_static"),
            ("/tf", "tf"),
        ],
    )

    return LaunchDescription(
        [
            *launch_args,
            robot_state_publisher_node,
            joint_state_publisher_node,
            tf,
        ]
    )
