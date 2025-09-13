from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_cpp import get_package_share_directory

def generate_launch_description():

    # パッケージのshareディレクトリを取得
    package_share_directory = get_package_share_directory('robot_config')

    return LaunchDescription([
        # DualArmTrajectoryRecorderノード
        Node(
            package='robot_config',
            executable='dual_arm_trajectory_recorder',
            name='dual_arm_trajectory_recorder',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
            remappings=[
                ('/joint_states', '/joint_states'),
                ('/trajectory_recorder/record', '/trajectory_recorder/record'),
                ('/trajectory_recorder/start', '/trajectory_recorder/start'),
                ('/trajectory_recorder/status', '/trajectory_recorder/status'),
            ]
        ),

        # move_to_pose_dual_cppノード（既存）
        Node(
            package='robot_config',
            executable='move_to_pose_dual_cpp',
            name='move_to_pose_dual_cpp',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        )
    ])