from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "description"


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        name="gui", description="Use gui gazebo", default_value="true"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", description="Use sim (gazebo) time", default_value="true"
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

    launch_args = [
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

    robot_description_content = Command(
        ["xacro ", xacro_file, " use_sim_time:=", use_sim_time]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_content,
            }
        ],
        output="both",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="both",
        condition=UnlessCondition(gui),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="both",
        condition=IfCondition(gui),
    )

    tf = Node(
        package=package_name,
        executable="tf_wrapper",
        name="static_transform_publisher",
        output="both",
        parameters=[{"config_file": tfconfig}],
    )

    joint_state_publisher_nodes = [
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
    ]

    delay_after_imu_relay = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[*joint_state_publisher_nodes, tf],
        )
    )

    return LaunchDescription(
        [
            *launch_args,
            robot_state_publisher_node,
            delay_after_imu_relay,
        ]
    )
