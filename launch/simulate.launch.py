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
    world_file_path = LaunchConfiguration("world", default="world_unconstrained.xml")
    #walking_impl_active = LaunchConfiguration("walk", default="False")

    mujoco_model_xml_path = PathJoinSubstitution(
        [
            FindPackageShare("solo_mujoco"),
            "mujoco",
            world_file_path
        ]
    )
    meshes_path = PathJoinSubstitution(
        [
            FindPackageShare("solo_mujoco"),
            "meshes"
        ]
    )

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
            "sim:=true" # Needed in URDF file to activate hardware plugin
            " ",
            "mujoco_world_xml_path:=", mujoco_model_xml_path,
            " ",
            "meshes_path:=",
            meshes_path
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
        #remappings=[
        #    ("~/robot_description", "/robot_description"),
        #],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            #"--controller-manager",
            #"/controller_manager",
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

    # Quad_walker implementation
    quad_walker = Node(
        package="solo_mujoco",
        executable="quad_walker",
        output="screen",
        condition=IfCondition(LaunchConfiguration('walk'))
    )

    # TaskController implementation
    task_controller = Node(
        package="solo_mujoco",
        executable="task_controller",
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument('walk', default_value='false'),
        DeclareLaunchArgument('plot', default_value='true'),

        ros2_control_node,
        rsp,
        joint_state_broadcaster_spawner,
        delay_position_controller_spawner_after_jsbs,
        delay_velocity_controller_spawner_after_jsbs,
        quad_walker,
        task_controller,
        rqt_plot
    ])
