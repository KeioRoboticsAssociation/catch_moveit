import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="robot_config")
        .robot_description(
            file_path="config/0525_arm.urdf.xacro",
            mappings={
                "use_fake_hardware": "false",
            },
        )
        .to_moveit_configs()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control_node
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("robot_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    # Spawner nodes
    spawn_controllers = []
    for controller in ["arm_controller", "hand_controller", "joint_state_broadcaster"]:
        spawn_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                output="screen",
            )
        )

    # Delay spawn_controllers until ros2_control_node is running
    delay_spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=spawn_controllers,
        )
    )

    # Servo node for realtime control
    servo_yaml = PathJoinSubstitution(
        [FindPackageShare("robot_config"), "config", "servo.yaml"]
    )
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            ros2_control_node,
            delay_spawn_controllers,
            servo_node,
        ]
    )
