from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = 'control'
package_name_description = 'description'


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', description='Use sim (gazebo) time', default_value='true'
    )
    control_file_arg = DeclareLaunchArgument(
        name='control_file',
        default_value='robot.yaml',
        description='Control file .yaml',
    )
    xacro_file_arg = DeclareLaunchArgument(
        name='xacro_file',
        description='Name of xacro file (if use_urdf=false)',
    )
    imu_filter_config_arg = DeclareLaunchArgument(
        name='imu_filter_config',
        default_value='sensors__imu_filter.yaml',
        description='Config file .yaml',
    )

    launch_args = [
        use_sim_time_arg,
        control_file_arg,
        xacro_file_arg,
        imu_filter_config_arg,
    ]
    control_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'config',
            LaunchConfiguration('control_file'),
        ]
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name_description),
            'urdf',
            LaunchConfiguration('xacro_file'),
        ]
    )
    imu_filter_config = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'config',
            LaunchConfiguration('imu_filter_config'),
        ]
    )

    robot_description_content = Command(
        ['xacro ', xacro_file, ' use_sim_time:=', use_sim_time]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_content, control_file],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--switch-timeout',
            '10',
        ],
    )

    robot_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'effort_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            control_file,
            '--switch-timeout',
            '10',
        ],
    )

    planner_node = Node(
        package=package_name,
        executable='planner_angle_node',
        name='planner_angle_error_node',
        namespace='planner',
    )

    angles_to_effort = Node(
        package=package_name,
        executable='angles_to_effort',
        name='angles_to_effort',
        namespace='planner',
    )

    delay_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    observation_prepare = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(package_name), 'launch', 'observation_prepare.launch.py']
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'imu_filter_config': imu_filter_config,
        }.items(),
    )

    delay_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                robot_position_controller_spawner,
                observation_prepare,
            ],
        )
    )

    delay_after_robot_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_position_controller_spawner,
            on_exit=[planner_node],
        )
    )

    delay_after_planner_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=planner_node,
            on_start=[angles_to_effort],
        )
    )

    return LaunchDescription(
        [
            *launch_args,
            ros2_control_node,
            delay_after_ros2_control_node,
            delay_after_joint_state_broadcaster_spawner,
            delay_after_robot_position_controller_spawner,
            delay_after_planner_node,
        ]
    )
