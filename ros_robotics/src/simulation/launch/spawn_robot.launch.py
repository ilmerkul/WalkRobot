from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "simulation"


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        name="gui", description="Use gui gazebo", default_value="true"
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", description="Name of robot", default_value="tropy_spot"
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
    control_file_arg = DeclareLaunchArgument(
        name="control_file",
        default_value="robot.yaml",
        description="Control file .yaml",
    )
    spawn_entity_config_arg = DeclareLaunchArgument(
        name="spawn_entity_config",
        default_value="spawn_entity.yaml",
        description="Spawn entity config .yaml",
    )

    launch_args = [
        gui_arg,
        robot_name_arg,
        use_sim_time_arg,
        tfconfig_arg,
        xacro_file_arg,
        control_file_arg,
        spawn_entity_config_arg,
    ]

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_file = LaunchConfiguration("xacro_file")
    control_file = LaunchConfiguration("control_file")
    spawn_entity_config = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            LaunchConfiguration("spawn_entity_config"),
        ]
    )
    tfconfig = LaunchConfiguration("tfconfig")

    spawn_robot = Node(
        package=package_name,
        executable="spawn_entity_wrapper",
        name="spawn_entity_wrapper",
        namespace="spawn_entity_wrapper",
        parameters=[
            {
                "spawn_config": spawn_entity_config,
            }
        ],
    )

    robot_state_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("description"), "launch", "state.launch.py"]
        ),
        launch_arguments={
            "gui": gui,
            "use_sim_time": use_sim_time,
            "tfconfig": tfconfig,
            "xacro_file": xacro_file,
        }.items(),
    )

    control_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("control"), "launch", "control.launch.py"]
        ),
        launch_arguments={
            "control_file": control_file,
            "use_sim_time": use_sim_time,
            "xacro_file": xacro_file,
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_args,
            spawn_robot,
            robot_state_launch,
            control_launch,
        ]
    )
