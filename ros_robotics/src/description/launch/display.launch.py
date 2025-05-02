from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    rvizconfig_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value="urdf.rviz",
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
        rvizconfig_arg,
        tfconfig_arg,
        xacro_file_arg,
    ]

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rvizconfig = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", LaunchConfiguration("rvizconfig")]
    )
    xacro_file = LaunchConfiguration("xacro_file")
    xacro_file_path = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "urdf",
            LaunchConfiguration("xacro_file"),
        ]
    )
    tfconfig = LaunchConfiguration("tfconfig")

    robot_description_content = Command(
        ["xacro ", xacro_file_path, " use_sim_time:=", use_sim_time]
    )

    publish_robot_description_node = Node(
        package=package_name,
        executable="robot_description_publisher",
        name="robot_description_publisher",
        output="both",
        arguments=[
            "-xml_string",
            robot_description_content,
            "-robot_description_topic",
            "/robot_description",
        ],
    )

    robot_state_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(package_name), "launch", "state.launch.py"]
        ),
        launch_arguments={
            "gui": gui,
            "use_sim_time": use_sim_time,
            "tfconfig": tfconfig,
            "xacro_file": xacro_file,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        name="rviz_node",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rvizconfig],
        condition=IfCondition(gui),
    )

    return LaunchDescription(
        [
            *launch_args,
            publish_robot_description_node,
            robot_state_launch,
            rviz_node,
        ]
    )
