#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "simulation"
package_description_name = "description"
package_control_name = "control"


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(name="gui", description="Use gui gazebo")
    name_arg = DeclareLaunchArgument(name="name", description="Name of robot")
    x_pose_arg = DeclareLaunchArgument(name="x_pose", description="x pose")
    y_pose_arg = DeclareLaunchArgument(name="y_pose", description="y pose")
    z_pose_arg = DeclareLaunchArgument(name="z_pose", description="z pose")
    Y_pose_arg = DeclareLaunchArgument(name="Y_pose", description="Y pose")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", description="Use sim (gazebo) time"
    )
    tfconfig_arg = DeclareLaunchArgument(
        name="tfconfig",
        description="Absolute path to tf config file",
    )
    xacro_file_arg = DeclareLaunchArgument(
        name="xacro_file",
        description="Name of xacro file (if use_urdf=false)",
    )
    control_file_arg = DeclareLaunchArgument(
        name="control_file",
        description="Control file .yaml",
    )

    launch_args = [
        gui_arg,
        name_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        Y_pose_arg,
        use_sim_time_arg,
        tfconfig_arg,
        xacro_file_arg,
        control_file_arg,
    ]

    gui = LaunchConfiguration("gui")
    name = LaunchConfiguration("name")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    Y_pose = LaunchConfiguration("Y_pose")
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_file = LaunchConfiguration("xacro_file")
    control_file = LaunchConfiguration("control_file")
    tfconfig = LaunchConfiguration("tfconfig")

    robot_description_namespace = [package_description_name, "robot_description"]
    robot_sensors_namespace = ["/", name, "sensors"]

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=name,
        arguments=[
            "-topic",
            PathJoinSubstitution(robot_description_namespace),
            "-name",
            name,
            "-allow_renaming",
            "true",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
            "-Y",
            Y_pose,
        ],
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=name,
        name="ros_gz_bridge",
        output="both",
        arguments=[
            PathJoinSubstitution(
                [*robot_sensors_namespace, "imu/raw@sensor_msgs/msg/Imu@gz.msgs.IMU"]
            ),
            PathJoinSubstitution(
                [
                    *robot_sensors_namespace,
                    "force_torque/back_left_foot_joint@geometry_msgs/msg/WrenchStamped@gz.msgs.Wrench",
                ]
            ),
            PathJoinSubstitution(
                [
                    *robot_sensors_namespace,
                    "force_torque/back_right_foot_joint@geometry_msgs/msg/WrenchStamped@gz.msgs.Wrench",
                ]
            ),
            PathJoinSubstitution(
                [
                    *robot_sensors_namespace,
                    "force_torque/front_left_foot_joint@geometry_msgs/msg/WrenchStamped@gz.msgs.Wrench",
                ]
            ),
            PathJoinSubstitution(
                [
                    *robot_sensors_namespace,
                    "force_torque/front_right_foot_joint@geometry_msgs/msg/WrenchStamped@gz.msgs.Wrench",
                ]
            ),
        ],
    )

    state_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(package_description_name), "launch", "state.launch.py"]
        ),
        launch_arguments={
            "gui": gui,
            "use_sim_time": use_sim_time,
            "tfconfig": tfconfig,
            "xacro_file": xacro_file,
            "robot_namespace": name,
        }.items(),
    )

    control_robot_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare(package_control_name),
                "launch",
                "control_robot.launch.py",
            ]
        ),
        launch_arguments={
            "control_file": control_file,
            "use_sim_time": use_sim_time,
            "robot_namespace": name,
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_args,
            spawn_entity,
            ros_gz_bridge,
            state_launch,
            control_robot_launch,
        ]
    )
