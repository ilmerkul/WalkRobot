#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare

package_name = "simulation"
package_control_name = "control"


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
    control_file_arg = DeclareLaunchArgument(
        name="control_file",
        default_value="control.yaml",
        description="Control file .yaml",
    )

    launch_args = [
        gui_arg,
        use_sim_time_arg,
        tfconfig_arg,
        xacro_file_arg,
        control_file_arg,
    ]

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_file = LaunchConfiguration("xacro_file")
    control_file = LaunchConfiguration("control_file")
    tfconfig = LaunchConfiguration("tfconfig")

    robots_file_path = os.path.join(
        get_package_share_directory(package_name), "config", "robots.yaml"
    )

    with open(robots_file_path, "r") as file:
        yaml_data = yaml.safe_load(file)
    robots = yaml_data["robots"]

    ld = []
    for i, robot in enumerate(robots):
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [FindPackageShare(package_name), "launch", "robot.launch.py"]
                    )
                ]
            ),
            launch_arguments={
                "gui": gui,
                "use_sim_time": use_sim_time,
                "tfconfig": tfconfig,
                "xacro_file": xacro_file,
                "control_file": control_file,
                "name": TextSubstitution(text=robot["name"]),
                "x_pose": TextSubstitution(text=robot["x_pose"]),
                "y_pose": TextSubstitution(text=robot["y_pose"]),
                "z_pose": TextSubstitution(text=robot["z_pose"]),
                "Y_pose": TextSubstitution(text=robot["Y_pose"]),
            }.items(),
        )

        robot_launch = TimerAction(period=10.0 * i, actions=[robot_launch])

        ld.append(robot_launch)

    control_robots_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare(package_control_name),
                "launch",
                "control_robots.launch.py",
            ]
        ),
        launch_arguments={
            "namespace": "",
        }.items(),
    )

    ld.append(control_robots_launch)

    return LaunchDescription(
        [
            *launch_args,
            *ld,
        ]
    )
