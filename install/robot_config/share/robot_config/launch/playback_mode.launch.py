#!/usr/bin/env python3
"""
Launch file for rosbag playback mode.
This disables joint_state_publisher to prevent conflicts.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "bag_file",
            description="Path to rosbag file to play",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rate", 
            default_value="0.5",
            description="Playback rate (default: 0.5 for safety)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "remap_topic",
            default_value="true",
            description="Remap /joint_states_recorded to /joint_states",
        )
    )

    # Initialize Arguments
    bag_file = LaunchConfiguration("bag_file")
    rate = LaunchConfiguration("rate")
    remap_topic = LaunchConfiguration("remap_topic")

    # Get MoveIt configs for dual arm
    moveit_config = (
        MoveItConfigsBuilder("dual_arm", package_name="robot_config")
        .robot_description(file_path="config/dual_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_arm.srdf")
        .robot_description_kinematics(file_path="config/dual_arm_kinematics.yaml")
        .joint_limits(file_path="config/dual_arm_joint_limits.yaml")
        .to_moveit_configs()
    )

    # Robot state publisher (NO joint_state_publisher to avoid conflicts)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher", 
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "left_base_link"],
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("robot_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Rosbag play with topic remapping
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, 
             '--rate', rate,
             '--remap', '/joint_states_recorded:=/joint_states'],
        output='screen',
        shell=False
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            static_tf,
            rviz_node,
            rosbag_play,
        ]
    )