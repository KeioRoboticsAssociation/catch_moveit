#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Launch Arguments
    control_type_arg = DeclareLaunchArgument(
        'control_type',
        default_value='cartesian',
        description='Type of control: "realtime" or "cartesian"'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='20.0',
        description='Control loop frequency in Hz'
    )
    
    velocity_scale_arg = DeclareLaunchArgument(
        'velocity_scale',
        default_value='0.1',
        description='Velocity scaling factor'
    )
    
    timeout_duration_arg = DeclareLaunchArgument(
        'timeout_duration',
        default_value='0.5',
        description='Command timeout duration in seconds'
    )

    # リアルタイム制御ノード
    dual_arm_realtime_control_node = Node(
        package='robot_config',
        executable='dual_arm_realtime_control',
        name='dual_arm_realtime_control',
        output='screen',
        condition=lambda context: LaunchConfiguration('control_type').perform(context) == 'realtime'
    )
    
    # Cartesian制御ノード
    dual_arm_cartesian_control_node = Node(
        package='robot_config',
        executable='dual_arm_cartesian_control',
        name='dual_arm_cartesian_control',
        output='screen',
        parameters=[{
            'control_frequency': LaunchConfiguration('control_frequency'),
            'velocity_scale': LaunchConfiguration('velocity_scale'),
            'timeout_duration': LaunchConfiguration('timeout_duration')
        }],
        condition=lambda context: LaunchConfiguration('control_type').perform(context) == 'cartesian'
    )

    return LaunchDescription([
        control_type_arg,
        control_frequency_arg,
        velocity_scale_arg,
        timeout_duration_arg,
        dual_arm_realtime_control_node,
        dual_arm_cartesian_control_node
    ])
