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
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robot_config"), "config", "moveit.rviz"]
            ),
            description="Path to RViz configuration file",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # Get MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder("0525_arm", package_name="robot_config")
        .robot_description(
            file_path="config/0525_arm.urdf.xacro",
            mappings={
                "use_fake_hardware": "true",
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

    # ##################################################################
    # ## 1. ros2_control_node を手動で起動 ##
    # ##################################################################
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

    # ##################################################################
    # ## 2. spawnerノードのリストを準備 ##
    # ##################################################################
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

    # ##################################################################
    # ## 3. イベントハンドラを定義 ##
    # ## ros2_control_node の起動が完了したら、spawnerを起動する ##
    # ##################################################################
    delay_spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=spawn_controllers,
        )
    )

    # MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-log-level", "info"],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            move_group_node,
            rviz_node,
            static_tf,
            # ros2_control_node と イベントハンドラを起動リストに追加
            ros2_control_node,
            delay_spawn_controllers,
        ]
    )