from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='d415_rgb_depth_3d',
            executable='d415_rgb_depth_3d_node',
            name='d415_rgb_depth_3d_node',
            output='screen',
            parameters=[{
                'color_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'projection_mode': 'fixed_world_z',
                'fixed_world_z': 0.086,
                'camera_position_x': 0.00, 
                'camera_position_y': -0.1795, 
                'camera_position_z': 0.5,
                'orientation_mode': 'rpy',
                'camera_roll_deg': -120.0,
                'camera_pitch_deg': 0.0,
                'camera_yaw_deg': 0.0,
                'target_frame': 'world',
                'hover_offset': 0.10,
            }]
        )
    ])