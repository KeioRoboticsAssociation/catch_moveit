import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Get MoveIt configs for dual arm
    moveit_config = (
        MoveItConfigsBuilder("dual_arm", package_name="robot_config")
        .robot_description(
            file_path="config/dual_arm.urdf.xacro",
            mappings={
                "use_fake_hardware": "true",
                "initial_positions_file": PathJoinSubstitution(
                    [FindPackageShare("robot_config"), "config", "initial_positions.yaml"]
                ),
            },
        )
        .robot_description_semantic(file_path="config/dual_arm.srdf")
        .robot_description_kinematics(file_path="config/dual_arm_kinematics.yaml")
        .joint_limits(file_path="config/dual_arm_joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .trajectory_execution(
            file_path="config/dual_arm_moveit_controllers.yaml",
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
            "dual_arm_controllers.yaml",
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
    for controller in ["left_arm_controller", "right_arm_controller", "left_hand_controller", "right_hand_controller", "joint_state_broadcaster"]:
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
            # Disable OctoMap updates
            {"planning_scene_monitor_options.publish_planning_scene": True},
            {"planning_scene_monitor_options.publish_geometry_updates": True},
            {"planning_scene_monitor_options.publish_state_updates": True},
            {"planning_scene_monitor_options.publish_transforms_updates": True},
            {"planning_scene_monitor_options.octomap_resolution": 0.0},
            # Set OMPL random seed for reproducible planning
            {"ompl/random_seed": 42},
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
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "left_base_link"],
    )

    # Servo node for realtime control (configured for left arm)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            {
                "moveit_servo": {
                    "use_gazebo": False,
                    "command_in_type": "speed_units",
                    "scale": {
                        "linear": 0.003,
                        "rotational": 0.006,
                        "joint": 0.01,
                    },
                    "low_latency_mode": False,
                    "publish_period": 0.01,
                    "command_out_type": "trajectory_msgs/JointTrajectory",
                    "publish_joint_positions": True,
                    "publish_joint_velocities": False,
                    "publish_joint_accelerations": False,
                    "smoothing_filter_plugin_name": "online_signal_smoothing::ButterworthFilterPlugin",
                    "move_group_name": "left_arm",
                    "planning_frame": "world",
                    "ee_frame_name": "left_EndEffector_1",
                    "robot_link_command_frame": "world",
                    "incoming_command_timeout": 1.0,
                    "num_outgoing_halt_msgs_to_publish": 1,
                    "lower_singularity_threshold": 30.0,
                    "hard_stop_singularity_threshold": 90.0,
                    "joint_limit_margin": 0.1,
                    "cartesian_command_in_topic": "servo_node/delta_twist_cmds",
                    "joint_command_in_topic": "servo_node/delta_joint_cmds",
                    "joint_topic": "joint_states",
                    "status_topic": "servo_node/status",
                    "command_out_topic": "/left_arm_controller/joint_trajectory",
                    "check_collisions": True,
                    "collision_check_rate": 5.0,
                    "self_collision_proximity_threshold": 0.01,
                    "scene_collision_proximity_threshold": 0.03,
                }
            },
        ],
        output="screen",
    )

    # Joy node for controller input
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    # Your teleop node
    teleop_node = Node(
        package="robot_config",
        executable="joystick_servo",
        name="joystick_servo",
        output="screen",
    )

    # C++ node for moving to a pose (single arm)
    # move_to_pose_cpp_node = Node(
    #     package="robot_config",
    #     executable="move_to_pose_cpp",
    #     name="move_to_pose_cpp",
    #     output="screen",
    #     # Pass the MoveIt-generated parameters to this node
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #     ],
    # )

    # C++ node for moving to a pose (dual arm)
    move_to_pose_dual_cpp_node = Node(
        package="robot_config",
        executable="move_to_pose_dual_cpp",
        name="move_to_pose_dual_cpp",
        output="screen",
        # Pass the MoveIt-generated parameters to this node
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Node to publish collision mesh
    publish_collision_mesh_node = Node(
        package="robot_config",
        executable="publish_collision_mesh.py",
        name="publish_collision_mesh",
        output="screen",
    )

    # Rosbridge server
    rosbridge_websocket = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        output="screen",
    )

    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        output="screen",
    )

    # NPM dev server
    npm_run_dev = ExecuteProcess(
        cmd=['npm', 'run', 'dev'],
        cwd='/home/a/ws_moveit2/src/robot_config/catch_gui',
        shell=True,
        output='screen'
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
            servo_node,
            joy_node,
            teleop_node,
            # move_to_pose_cpp_node,
            move_to_pose_dual_cpp_node, # Add the new dual arm node
            publish_collision_mesh_node,
            # Additional nodes
            rosbridge_websocket,
            rosapi_node,
            npm_run_dev,
        ]
    )