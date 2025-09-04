from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare camera position arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_position_x",
            default_value="0.00",
            description="X position of camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_position_y", 
            default_value="-0.1795",
            description="Y position of camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_position_z",
            default_value="1.0",
            description="Z position of camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_yaw_deg",
            default_value="0.0",
            description="Yaw angle of camera in degrees",
        )
    )
    
    # Get launch configurations
    camera_position_x = LaunchConfiguration("camera_position_x")
    camera_position_y = LaunchConfiguration("camera_position_y")
    camera_position_z = LaunchConfiguration("camera_position_z")
    camera_yaw_deg = LaunchConfiguration("camera_yaw_deg")
    
    return LaunchDescription(
        declared_arguments + [
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
                    'fixed_world_z': 0.2,
                    'camera_position_x': camera_position_x, 
                    'camera_position_y': camera_position_y, 
                    'camera_position_z': camera_position_z,
                    'orientation_mode': 'rpy',
                    'camera_roll_deg': 0.0,
                    'camera_pitch_deg': 150.0,
                    'camera_yaw_deg': camera_yaw_deg,
                    'target_frame': 'world',
                    'hover_offset': 0.10,
                }]
            )
        ]
    )