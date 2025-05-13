from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "control"


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", description="Use sim (gazebo) time", default_value="true"
    )
    imu_filter_config_arg = DeclareLaunchArgument(
        name="imu_filter_config",
        default_value="sensors__imu_filter.yaml",
        description="Config file .yaml",
    )

    launch_args = [
        use_sim_time_arg,
        imu_filter_config_arg,
    ]
    imu_filter_config = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            LaunchConfiguration("imu_filter_config"),
        ]
    )

    imu_relay = Node(
        package="topic_tools",
        executable="relay",
        name="imu_topic_relay",
        parameters=[
            {"input_topic": "/imu_plugin/out", "output_topic": "/sensors/imu/data_raw"}
        ],
    )

    imu_filter_madgwick = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        namespace="sensors",
        output="both",
        parameters=[imu_filter_config],
        remappings=[
            # Входные топики (подписка фильтра)
            ("imu/data_raw", "/sensors/imu/data_raw"),  # Сырые данные IMU
            ("imu/mag", "/sensors/imu/mag/data"),  # Данные магнитометра (если есть)
            # Выходной топик (публикация фильтра)
            ("imu/data", "/sensors/imu/filtered"),  # Отфильтрованные данные
        ],
    )

    foot_contact_detector_node = Node(
        package=package_name,
        executable="foot_contact_detector",
        name="foot_contact_detector",
        namespace="sensors",
    )

    observation_aggregator = Node(
        package=package_name,
        executable="observation_aggregator",
        name="observation_aggregator",
        namespace="observation",
    )

    delay_after_imu_relay = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=imu_relay,
            on_start=[imu_filter_madgwick],
        )
    )

    return LaunchDescription(
        [
            *launch_args,
            foot_contact_detector_node,
            imu_relay,
            delay_after_imu_relay,
            observation_aggregator,
        ]
    )
