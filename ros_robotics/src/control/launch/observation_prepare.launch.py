from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "control"
package_name_description = "description"


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", description="Use sim (gazebo) time", default_value="true"
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
        imu_filter_config_arg,
        robot_namespace_arg,
    ]
    imu_filter_config = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            LaunchConfiguration("imu_filter_config"),
        ]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")

    robot_sensors_namespace = PythonExpression(
        ["'/' + '", robot_namespace, "' + '/sensors'"]
    )
    robot_description_namespace = PythonExpression(
        ["'/' + '", robot_namespace, f"' + '/{package_name_description}'"]
    )

    imu_filter_madgwick = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        namespace=robot_sensors_namespace,
        output="both",
        parameters=[
            {"use_mag": False, "use_sim_time": use_sim_time},
            imu_filter_config,
        ],
        remappings=[
            ("imu/data_raw", "imu/raw"),
            ("imu/data", "imu/filtered"),
            (
                "/tf",
                PythonExpression(["'", robot_description_namespace, f"' + '/tf'"]),
            ),
        ],
    )

    foot_contact_detector_node = Node(
        package=package_name,
        executable="foot_contact_detector",
        name="foot_contact_detector",
        namespace=robot_namespace,
    )

    observation_aggregator = Node(
        package=package_name,
        executable="observation_aggregator",
        name="observation_aggregator",
        namespace=robot_namespace,
    )

    return LaunchDescription(
        [
            *launch_args,
            foot_contact_detector_node,
            imu_filter_madgwick,
            observation_aggregator,
        ]
    )
