from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "control"


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", description="Use sim (gazebo) time", default_value="true"
    )
    control_file_arg = DeclareLaunchArgument(
        name="control_file",
        default_value="control.yaml",
        description="Control file .yaml",
    )
    imu_filter_config_arg = DeclareLaunchArgument(
        name="imu_filter_config",
        default_value="sensors__imu_filter.yaml",
        description="Config file .yaml",
    )
    robot_namespace_arg = DeclareLaunchArgument(
        name="robot_namespace",
        default_value="tropy_spot",
        description="robot namespace",
    )

    launch_args = [
        use_sim_time_arg,
        control_file_arg,
        imu_filter_config_arg,
        robot_namespace_arg,
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_filter_config = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            LaunchConfiguration("imu_filter_config"),
        ]
    )
    robot_namespace = LaunchConfiguration("robot_namespace")
    control_file = LaunchConfiguration("control_file")

    robot_package_namespace = PythonExpression(
        ["'/' + '", robot_namespace, f"' + '/{package_name}'"]
    )

    if True:
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
    elif False:
        effort_control = IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "launch",
                    "controller_manager.launch.py",
                ]
            ),
            launch_arguments={
                "robot_namespace": robot_namespace,
                "use_sim_time": use_sim_time,
                "control_file": control_file,
            }.items(),
        )

    observation_prepare = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(package_name), "launch", "observation_prepare.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "imu_filter_config": imu_filter_config,
            "robot_namespace": robot_namespace,
        }.items(),
    )

    angles_to_effort_node = Node(
        package=package_name,
        executable="angles_to_effort_node",
        name="angles_to_effort_node",
        namespace=robot_package_namespace,
    )

    angles_to_effort_node_timer = TimerAction(
        period=9.0,
        actions=[angles_to_effort_node],
    )

    return LaunchDescription(
        [
            *launch_args,
            effort_control,
            observation_prepare,
            angles_to_effort_node_timer,
        ]
    )
