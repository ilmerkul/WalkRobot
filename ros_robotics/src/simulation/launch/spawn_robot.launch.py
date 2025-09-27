from control.utils import modify_namespace_and_save_yaml_files
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnExecutionComplete, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

package_name = "simulation"


def generate_robot_launchs(context, *args, **kwargs):
    actions = []
    base_namespace = context.launch_configurations["robot_namespace"]
    entity_count = int(context.launch_configurations["entity_count"])

    modify_namespace_and_save_yaml_files(
        context.launch_configurations["control_file"], entity_count, base_namespace
    )

    prev_action = args[0]
    for i in range(entity_count):
        robot_state_launch = IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("description"), "launch", "state.launch.py"]
            ),
            launch_arguments={
                "gui": context.launch_configurations["gui"],
                "use_sim_time": context.launch_configurations["use_sim_time"],
                "tfconfig": context.launch_configurations["tfconfig"],
                "xacro_file": context.launch_configurations["xacro_file"],
                "robot_namespace": f"{base_namespace}_{i}",
            }.items(),
        )

        control_launch = IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("control"), "launch", "control.launch.py"]
            ),
            launch_arguments={
                "control_file": context.launch_configurations["control_file"],
                "use_sim_time": context.launch_configurations["use_sim_time"],
                "robot_namespace": f"{base_namespace}_{i}",
            }.items(),
        )
        actions.append(
            RegisterEventHandler(
                OnExecutionComplete(
                    target_action=prev_action, on_completion=[robot_state_launch]
                )
            )
        )
        actions.append(
            RegisterEventHandler(
                OnExecutionComplete(
                    target_action=robot_state_launch, on_completion=[control_launch]
                )
            )
        )

        prev_action = control_launch

    return actions


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
        default_value="control.yaml",
        description="Control file .yaml",
    )
    spawn_entity_config_arg = DeclareLaunchArgument(
        name="spawn_entity_config",
        default_value="spawn_entity.yaml",
        description="Spawn entity config .yaml",
    )
    entity_count_arg = DeclareLaunchArgument(
        name="entity_count",
        default_value="1",
        description="Spawn entity count",
    )
    robot_namespace_arg = DeclareLaunchArgument(
        name="robot_namespace",
        default_value="tropy_spot",
        description="robot namespace",
    )

    launch_args = [
        gui_arg,
        robot_name_arg,
        use_sim_time_arg,
        tfconfig_arg,
        xacro_file_arg,
        control_file_arg,
        spawn_entity_config_arg,
        entity_count_arg,
        robot_namespace_arg,
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
    entity_count = LaunchConfiguration("entity_count")
    robot_namespace = LaunchConfiguration("robot_namespace")

    spawn_node = LifecycleNode(
        package=package_name,
        executable="spawn_entity",
        name="spawn_entity",
        namespace="spawn",
        parameters=[
            {
                "spawn_config": spawn_entity_config,
                "xacro_file": xacro_file,
                "entity_count": entity_count,
                "base_namespace": robot_namespace,
                "control_file": control_file,
                "gui": gui,
                "use_sim_time": use_sim_time,
                "tfconfig": tfconfig,
            }
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == spawn_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == spawn_node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[configure_event],
                )
            ],
        )
    )

    activate_event_handler = RegisterEventHandler(
        OnExecutionComplete(
            target_action=configure_event, on_completion=[activate_event]
        )
    )

    return LaunchDescription(
        [
            *launch_args,
            spawn_node,
            configure_event_handler,
            activate_event_handler,
        ]
    )
