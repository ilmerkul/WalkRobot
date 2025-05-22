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
    imu_filter_config_arg = DeclareLaunchArgument(
        name="imu_filter_config",
        default_value="sensors__imu_filter.yaml",
        description="Config file .yaml",
    )
    entity_count_arg = DeclareLaunchArgument(
        name="entity_count",
        default_value="1",
        description="Spawn entity count",
    )

    launch_args = [
        use_sim_time_arg,
        control_file_arg,
        xacro_file_arg,
        imu_filter_config_arg,
        entity_count_arg,
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_filter_config = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            LaunchConfiguration("imu_filter_config"),
        ]
    )

    effort_control = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare(package_name),
                "launch",
                "effort_control.launch.py",
            ]
        ),
    )

    angles_to_effort_node = Node(
        package=package_name,
        executable="angles_to_effort_node",
        name="angles_to_effort_node",
        namespace="/tropy_spot_0/control",
    )

    observation_prepare = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(package_name), "launch", "observation_prepare.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "imu_filter_config": imu_filter_config,
        }.items(),
    )

    stand_up_nn = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare(package_name_dl_control),
                "launch",
                "stand_up_nn.launch.py",
            ]
        ),
    )

    planner = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(package_name_planner), "launch", "planner.launch.py"]
        ),
    )

    return LaunchDescription(
        [
            *launch_args,
            observation_prepare,
            effort_control,
            planner,
            stand_up_nn,
            angles_to_effort_node,
        ]
    )
