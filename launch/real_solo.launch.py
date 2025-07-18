from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("solo_mujoco"),
                    "urdf",
                    "solo8.urdf.xacro"
                ]
            ),
            " ",
            "hardware_type:=real" # Needed in URDF file to activate hardware plugin. Choose 'sim' or 'real'
            " "
            "eth_interface:=", "eth0"
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("solo_mujoco"),
            "config",
            "controllers.yaml"
        ]
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers_config,
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ],
    )
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "--param-file",
            robot_controllers_config
        ],
    )
    delay_position_controller_spawner_after_jsbs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner]
        )
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
            "--param-file",
            robot_controllers_config
        ],
    )
    delay_velocity_controller_spawner_after_jsbs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner]
        )
    )

    rqt_plot = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot'],
        condition=IfCondition(LaunchConfiguration('plot'))
    )

    # TaskController implementation
    task_controller = Node(
        package="solo_mujoco",
        executable="task_controller",
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument('plot', default_value='false'),

        ros2_control_node,
        rsp,
        joint_state_broadcaster_spawner,
        delay_position_controller_spawner_after_jsbs,
        delay_velocity_controller_spawner_after_jsbs,
        task_controller,
        rqt_plot
    ])
