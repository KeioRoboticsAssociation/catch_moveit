import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


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
    declared_arguments.append(
        DeclareLaunchArgument(
            "field",
            default_value="red",
            description="Field type: 'red' for red team position, 'blue' for blue team position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_dual_controller",
            default_value="false",
            description="If true, spawn dual_arm_controller and use merged trajectory execution",
        )
    )
    # Declare arguments for left arm initial positions
    # Declare arguments for left arm joint limits
    
    # Declare arguments for left arm joint limits
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_1_lower_limit",
            default_value="-3.14",
            description="Lower limit for left arm Revolute_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_1_upper_limit",
            default_value="3.14",
            description="Upper limit for left arm Revolute_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_2_lower_limit",
            default_value="-1.570796",
            description="Lower limit for left arm Revolute_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_2_upper_limit",
            default_value="1.570796",
            description="Upper limit for left arm Revolute_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_3_lower_limit",
            default_value="-1.570796",
            description="Lower limit for left arm Revolute_3 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_3_upper_limit",
            default_value="1.570796",
            description="Upper limit for left arm Revolute_3 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Slider_1_lower_limit",
            default_value="0.0",
            description="Lower limit for left arm Slider_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Slider_1_upper_limit",
            default_value="0.024",
            description="Upper limit for left arm Slider_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Slider_2_lower_limit",
            default_value="-0.024",
            description="Lower limit for left arm Slider_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Slider_2_upper_limit",
            default_value="0.0",
            description="Upper limit for left arm Slider_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_4_lower_limit",
            default_value="0",
            description="Lower limit for left arm Revolute_4 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_4_upper_limit",
            default_value="0",
            description="Upper limit for left arm Revolute_4 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_6_lower_limit",
            default_value="-3.14",
            description="Lower limit for left arm Revolute_6 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_6_upper_limit",
            default_value="3.14",
            description="Upper limit for left arm Revolute_6 joint",
        )
    )
    
    # Declare arguments for right arm joint limits
    
    # Declare arguments for left arm joint origins
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_1_xyz",
            default_value="0.0 0.0 0.0555",
            description="XYZ origin for left arm Revolute_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_1_rpy",
            default_value="0 0 1.57",
            description="RPY origin for left arm Revolute_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_2_xyz",
            default_value="-3.7e-05 0.024464 0.06805",
            description="XYZ origin for left arm Revolute_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_2_rpy",
            default_value="0 0 0",
            description="RPY origin for left arm Revolute_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_3_xyz",
            default_value="0.034861 -0.011969 0.43175",
            description="XYZ origin for left arm Revolute_3 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_3_rpy",
            default_value="0 0 0",
            description="RPY origin for left arm Revolute_3 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_4_xyz",
            default_value="0.420611 -0.012207 -0.0001",
            description="XYZ origin for left arm Revolute_4 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_4_rpy",
            default_value="0 0 0",
            description="RPY origin for left arm Revolute_4 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_5_xyz",
            default_value="0.078583 0.01897 0.0",
            description="XYZ origin for left arm Revolute_5 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_5_rpy",
            default_value="0 0 0",
            description="RPY origin for left arm Revolute_5 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_6_xyz",
            default_value="1.7e-05 -0.0189 -0.0648",
            description="XYZ origin for left arm Revolute_6 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Revolute_6_rpy",
            default_value="0 0 0",
            description="RPY origin for left arm Revolute_6 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Rigid_7_xyz",
            default_value="-5e-05 0.0 -0.045",
            description="XYZ origin for left arm Rigid_7 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Rigid_7_rpy",
            default_value="0 0 0",
            description="RPY origin for left arm Rigid_7 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Slider_1_xyz",
            default_value="0.055 7.3e-05 -0.008",
            description="XYZ origin for left arm Slider_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Slider_1_rpy",
            default_value="0 0 0",
            description="RPY origin for left arm Slider_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Slider_2_xyz",
            default_value="-0.055 -7.2e-05 -0.008",
            description="XYZ origin for left arm Slider_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_Slider_2_rpy",
            default_value="0 0 0",
            description="RPY origin for left arm Slider_2 joint",
        )
    )
    
    # Declare arguments for right arm joint origins
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_1_xyz",
            default_value="0.0 0.0 0.0555",
            description="XYZ origin for right arm Revolute_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_1_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm Revolute_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_2_xyz",
            default_value="-3.7e-05 0.024464 0.06805",
            description="XYZ origin for right arm Revolute_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_2_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm Revolute_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_3_xyz",
            default_value="0.034861 -0.011969 0.43175",
            description="XYZ origin for right arm Revolute_3 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_3_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm Revolute_3 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_4_xyz",
            default_value="0.420611 -0.012207 -0.0001",
            description="XYZ origin for right arm Revolute_4 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_4_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm Revolute_4 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_5_xyz",
            default_value="0.078583 0.01897 0.0",
            description="XYZ origin for right arm Revolute_5 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_5_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm Revolute_5 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_6_xyz",
            default_value="1.7e-05 -0.0189 -0.0648",
            description="XYZ origin for right arm Revolute_6 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_6_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm Revolute_6 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Rigid_7_xyz",
            default_value="-5e-05 0.0 -0.045",
            description="XYZ origin for right arm Rigid_7 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Rigid_7_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm Rigid_7 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Slider_1_xyz",
            default_value="0.055 7.3e-05 -0.008",
            description="XYZ origin for right arm Slider_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Slider_1_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm Slider_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Slider_2_xyz",
            default_value="-0.055 -7.2e-05 -0.008",
            description="XYZ origin for right arm Slider_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Slider_2_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm Slider_2 joint",
        )
    )
    
    # Declare arguments for arm origins
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_origin_xyz",
            default_value="0 0.2 0",
            description="XYZ origin for left arm base (red team default)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_origin_rpy",
            default_value="0 0 0",
            description="RPY origin for left arm base",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_origin_xyz",
            default_value="0 -0.2 0",
            description="XYZ origin for right arm base (red team default)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_origin_rpy",
            default_value="0 0 0",
            description="RPY origin for right arm base",
        )
    )
    
    # Declare arguments for right arm joint limits
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_1_lower_limit",
            default_value="-3.141592",
            description="Lower limit for right arm Revolute_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_1_upper_limit",
            default_value="3.141592",
            description="Upper limit for right arm Revolute_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_2_lower_limit",
            default_value="-1.570796",
            description="Lower limit for right arm Revolute_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_2_upper_limit",
            default_value="1.570796",
            description="Upper limit for right arm Revolute_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_3_lower_limit",
            default_value="-1.570796",
            description="Lower limit for right arm Revolute_3 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_3_upper_limit",
            default_value="1.570796",
            description="Upper limit for right arm Revolute_3 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Slider_1_lower_limit",
            default_value="0.0",
            description="Lower limit for right arm Slider_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Slider_1_upper_limit",
            default_value="0.024",
            description="Upper limit for right arm Slider_1 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Slider_2_lower_limit",
            default_value="-0.024",
            description="Lower limit for right arm Slider_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Slider_2_upper_limit",
            default_value="0.0",
            description="Upper limit for right arm Slider_2 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_4_lower_limit",
            default_value="0",
            description="Lower limit for right arm Revolute_4 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_4_upper_limit",
            default_value="0",
            description="Upper limit for right arm Revolute_4 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_6_lower_limit",
            default_value="-3.14",
            description="Lower limit for right arm Revolute_6 joint",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_Revolute_6_upper_limit",
            default_value="3.14",
            description="Upper limit for right arm Revolute_6 joint",
        )
    )
    
    # Declare argument for box coordinates (24 coordinates per box: 8 corner points with x,y,z each)
    declared_arguments.append(
        DeclareLaunchArgument(
            "box_coordinates",
            default_value="0.134,-0.05528,-0.001,0.134,-0.05528,1.0,0.134,-0.05528,-0.001,0.134,-0.05528,1.0,0.564494,-0.05528,-0.001,0.564494,-0.05528,1.0,0.564494,-0.05528,-0.001,0.564494,-0.05528,1.0",
            description="Box coordinates: 24 values per box (8 corner points with x,y,z each). Must be multiple of 24 to create multiple boxes.",
        )
    )

    # Initialize Arguments FIRST
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    field = LaunchConfiguration("field")
    box_coordinates = LaunchConfiguration("box_coordinates")
    
    # Initialize arguments for left arm joint limits
    left_Revolute_1_lower_limit = LaunchConfiguration("left_Revolute_1_lower_limit")
    left_Revolute_1_upper_limit = LaunchConfiguration("left_Revolute_1_upper_limit")
    left_Revolute_2_lower_limit = LaunchConfiguration("left_Revolute_2_lower_limit")
    left_Revolute_2_upper_limit = LaunchConfiguration("left_Revolute_2_upper_limit")
    left_Revolute_3_lower_limit = LaunchConfiguration("left_Revolute_3_lower_limit")
    left_Revolute_3_upper_limit = LaunchConfiguration("left_Revolute_3_upper_limit")
    left_Slider_1_lower_limit = LaunchConfiguration("left_Slider_1_lower_limit")
    left_Slider_1_upper_limit = LaunchConfiguration("left_Slider_1_upper_limit")
    left_Slider_2_lower_limit = LaunchConfiguration("left_Slider_2_lower_limit")
    left_Slider_2_upper_limit = LaunchConfiguration("left_Slider_2_upper_limit")
    left_Revolute_4_lower_limit = LaunchConfiguration("left_Revolute_4_lower_limit")
    left_Revolute_4_upper_limit = LaunchConfiguration("left_Revolute_4_upper_limit")
    left_Revolute_6_lower_limit = LaunchConfiguration("left_Revolute_6_lower_limit")
    left_Revolute_6_upper_limit = LaunchConfiguration("left_Revolute_6_upper_limit")
    
    # Initialize arguments for right arm joint limits
    right_Revolute_1_lower_limit = LaunchConfiguration("right_Revolute_1_lower_limit")
    right_Revolute_1_upper_limit = LaunchConfiguration("right_Revolute_1_upper_limit")
    right_Revolute_2_lower_limit = LaunchConfiguration("right_Revolute_2_lower_limit")
    right_Revolute_2_upper_limit = LaunchConfiguration("right_Revolute_2_upper_limit")
    right_Revolute_3_lower_limit = LaunchConfiguration("right_Revolute_3_lower_limit")
    right_Revolute_3_upper_limit = LaunchConfiguration("right_Revolute_3_upper_limit")
    right_Slider_1_lower_limit = LaunchConfiguration("right_Slider_1_lower_limit")
    right_Slider_1_upper_limit = LaunchConfiguration("right_Slider_1_upper_limit")
    right_Slider_2_lower_limit = LaunchConfiguration("right_Slider_2_lower_limit")
    right_Slider_2_upper_limit = LaunchConfiguration("right_Slider_2_upper_limit")
    right_Revolute_4_lower_limit = LaunchConfiguration("right_Revolute_4_lower_limit")
    right_Revolute_4_upper_limit = LaunchConfiguration("right_Revolute_4_upper_limit")
    right_Revolute_6_lower_limit = LaunchConfiguration("right_Revolute_6_lower_limit")
    right_Revolute_6_upper_limit = LaunchConfiguration("right_Revolute_6_upper_limit")
    
    # Initialize arguments for left arm joint origins
    left_Revolute_1_xyz = LaunchConfiguration("left_Revolute_1_xyz")
    left_Revolute_1_rpy = PythonExpression([
        '"0 0 1.57" if "', field, '" == "red" else ',
        '"0 0 1.57" if "', field, '" == "blue" else "',
        LaunchConfiguration("left_Revolute_1_rpy"), '"'
    ])
    left_Revolute_2_xyz = LaunchConfiguration("left_Revolute_2_xyz")
    left_Revolute_2_rpy = LaunchConfiguration("left_Revolute_2_rpy")
    left_Revolute_3_xyz = LaunchConfiguration("left_Revolute_3_xyz")
    left_Revolute_3_rpy = LaunchConfiguration("left_Revolute_3_rpy")
    left_Revolute_4_xyz = LaunchConfiguration("left_Revolute_4_xyz")
    left_Revolute_4_rpy = LaunchConfiguration("left_Revolute_4_rpy")
    left_Revolute_5_xyz = LaunchConfiguration("left_Revolute_5_xyz")
    left_Revolute_5_rpy = LaunchConfiguration("left_Revolute_5_rpy")
    left_Revolute_6_xyz = LaunchConfiguration("left_Revolute_6_xyz")
    left_Revolute_6_rpy = LaunchConfiguration("left_Revolute_6_rpy")
    left_Rigid_7_xyz = LaunchConfiguration("left_Rigid_7_xyz")
    left_Rigid_7_rpy = LaunchConfiguration("left_Rigid_7_rpy")
    left_Slider_1_xyz = LaunchConfiguration("left_Slider_1_xyz")
    left_Slider_1_rpy = LaunchConfiguration("left_Slider_1_rpy")
    left_Slider_2_xyz = LaunchConfiguration("left_Slider_2_xyz")
    left_Slider_2_rpy = LaunchConfiguration("left_Slider_2_rpy")
    
    # Initialize arguments for right arm joint origins
    right_Revolute_1_xyz = LaunchConfiguration("right_Revolute_1_xyz")
    right_Revolute_1_rpy = PythonExpression([
        '"0 0 -1.57" if "', field, '" == "red" else ',
        '"0 0 -1.57" if "', field, '" == "blue" else "',
        LaunchConfiguration("right_Revolute_1_rpy"), '"'
    ])
    right_Revolute_2_xyz = LaunchConfiguration("right_Revolute_2_xyz")
    right_Revolute_2_rpy = LaunchConfiguration("right_Revolute_2_rpy")
    right_Revolute_3_xyz = LaunchConfiguration("right_Revolute_3_xyz")
    right_Revolute_3_rpy = LaunchConfiguration("right_Revolute_3_rpy")
    right_Revolute_4_xyz = LaunchConfiguration("right_Revolute_4_xyz")
    right_Revolute_4_rpy = LaunchConfiguration("right_Revolute_4_rpy")
    right_Revolute_5_xyz = LaunchConfiguration("right_Revolute_5_xyz")
    right_Revolute_5_rpy = LaunchConfiguration("right_Revolute_5_rpy")
    right_Revolute_6_xyz = LaunchConfiguration("right_Revolute_6_xyz")
    right_Revolute_6_rpy = LaunchConfiguration("right_Revolute_6_rpy")
    right_Rigid_7_xyz = LaunchConfiguration("right_Rigid_7_xyz")
    right_Rigid_7_rpy = LaunchConfiguration("right_Rigid_7_rpy")
    right_Slider_1_xyz = LaunchConfiguration("right_Slider_1_xyz")
    right_Slider_1_rpy = LaunchConfiguration("right_Slider_1_rpy")
    right_Slider_2_xyz = LaunchConfiguration("right_Slider_2_xyz")
    right_Slider_2_rpy = LaunchConfiguration("right_Slider_2_rpy")
    
    # Set arm positions dynamically based on field parameter
    left_origin_xyz = PythonExpression([
        '"1.62 -0.359 0" if "', field, '" == "red" else ',
        '"0 0 0" if "', field, '" == "blue" else "', 
        LaunchConfiguration("left_origin_xyz"), '"'
    ])
    
    right_origin_xyz = PythonExpression([
        '"1.62 0 0" if "', field, '" == "red" else ',
        '"0 -0.359 0" if "', field, '" == "blue" else "',
        LaunchConfiguration("right_origin_xyz"), '"'
    ])
    
    left_origin_rpy = PythonExpression([
        '"0 0 3.141592" if "', field, '" == "red" else ',
        '"0 0 0" if "', field, '" == "blue" else "',
        LaunchConfiguration("left_origin_rpy"), '"'
    ])
    
    right_origin_rpy = PythonExpression([
        '"0 0 -3.141592" if "', field, '" == "red" else ',
        '"0 0 0" if "', field, '" == "blue" else "',
        LaunchConfiguration("right_origin_rpy"), '"'
    ])
    
    # Define object positions for different field types
    def get_object_positions(field_value):
        """Get object positions based on field parameter"""
        red_positions = [-0.3,1.1,0,0,-0.2,1.1,0,0,-0.1,1.1,0,0,0,1.1,0,0,-0.3,1,0,0,-0.2,1,0,0,-0.1,1,0,0,0,1,0,0,-0.3,0.85,0,0,-0.2,0.85,0,0,-0.1,0.85,0,0,0,0.85,0,0,-0.3,0.75,0,0,-0.2,0.75,0,0,-0.1,0.75,0,0,0,0.75,0,0,-0.3,0.6,0,0,-0.2,0.6,0,0,-0.1,0.6,0,0,0,0.6,0,0,-0.3,0.5,0,0,-0.2,0.5,0,0,-0.1,0.5,0,0,0,0.5,0,0,-0.3,0.35,0,0,-0.2,0.35,0,0,-0.1,0.35,0,0,0,0.35,0,0,-0.3,0.25,0,0,-0.2,0.25,0,0,-0.1,0.25,0,0,0,0.25,0,0,-0.3,0.1,0,0,-0.2,0.1,0,0,-0.1,0.1,0,0,0,0.1,0,0,-0.3,0,0,0,-0.2,0,0,0,-0.1,0,0,0,0,0,0,0,-0.528,0.0175,0.003,0,-0.628,0.0175,0.003,0,-0.528,0.1175,0.003,0,-0.628,0.1175,0.003,0,-0.528,0.2175,0.003,0,-0.628,0.2175,0.003,0,-0.528,0.3175,0.003,0,-0.628,0.3175,0.003,0,-0.528,0.4175,0.003,0,-0.628,0.4175,0.003,0,-0.528,0.6825,0.003,0,-0.628,0.6825,0.003,0,-0.528,0.6825,0.003,0,-0.628,0.7825,0.003,0,-0.528,0.7825,0.003,0,-0.628,0.8825,0.003,0,-0.528,0.8825,0.003,0,-0.628,0.9825,0.003,0,-0.528,0.9825,0.003,0,-0.628,1.0825,0.003,0,-0.528,1.0825,0.003,0]
        
        blue_positions = [-1.156,1.1,0,0,-1.056,1.1,0,0,-0.956,1.1,0,0,-0.856,1.1,0,0,-1.156,1,0,0,-1.056,1,0,0,-0.956,1,0,0,-0.856,1,0,0,-1.156,0.85,0,0,-1.056,0.85,0,0,-0.956,0.85,0,0,-0.856,0.85,0,0,-1.156,0.75,0,0,-1.056,0.75,0,0,-0.956,0.75,0,0,-0.856,0.75,0,0,-1.156,0.6,0,0,-1.056,0.6,0,0,-0.956,0.6,0,0,-0.856,0.6,0,0,-1.156,0.5,0,0,-1.056,0.5,0,0,-0.956,0.5,0,0,-0.856,0.5,0,0,-1.156,0.35,0,0,-1.056,0.35,0,0,-0.956,0.35,0,0,-0.856,0.35,0,0,-1.156,0.25,0,0,-1.056,0.25,0,0,-0.956,0.25,0,0,-0.856,0.25,0,0,-1.156,0.1,0,0,-1.056,0.1,0,0,-0.956,0.1,0,0,-0.856,0.1,0,0,-1.156,0,0,0,-1.056,0,0,0,-0.956,0,0,0,-0.856,0,0,0,-0.528,0.0175,0.003,0,-0.628,0.0175,0.003,0,-0.528,0.1175,0.003,0,-0.628,0.1175,0.003,0,-0.528,0.2175,0.003,0,-0.628,0.2175,0.003,0,-0.528,0.3175,0.003,0,-0.628,0.3175,0.003,0,-0.528,0.4175,0.003,0,-0.628,0.4175,0.003,0,-0.528,0.6825,0.003,0,-0.628,0.6825,0.003,0,-0.528,0.6825,0.003,0,-0.628,0.7825,0.003,0,-0.528,0.7825,0.003,0,-0.628,0.8825,0.003,0,-0.528,0.8825,0.003,0,-0.628,0.9825,0.003,0,-0.528,0.9825,0.003,0,-0.628,1.0825,0.003,0,-0.528,1.0825,0.003,0]
        
        if field_value == "blue":
            return blue_positions
        else:
            return red_positions
    
    # Get field value (need to extract from LaunchConfiguration)
    import os
    field_value = os.environ.get('FIELD', 'red')  # Default to red
    
    # Define box coordinates for different field types
    def get_box_coordinates(field_value):
        """Get box coordinates based on field parameter"""
        # Current box_coordinates split into blue (first 24) and red (last 24)
        full_coordinates = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        blue_coordinates = full_coordinates  # First 24 values for blue
        red_coordinates = full_coordinates  # Last 24 values for red
        
        if field_value == "blue":
            return blue_coordinates
        else:
            return red_coordinates
    
    # Get box_coordinates value from command line (or use field-based default)
    box_coordinates_value = None  # Will be set based on field or command line
    
    # Check command line arguments for both field and box_coordinates parameters
    import sys
    for arg in sys.argv:
        if arg.startswith('field:='):
            field_value = arg.split(':=')[1]
        elif arg.startswith('box_coordinates:='):
            box_coordinates_value = arg.split(':=')[1]
    
    # If no box_coordinates specified via command line, use field-based default
    if box_coordinates_value is None:
        box_coordinates_list = get_box_coordinates(field_value)
    else:
        # Convert box_coordinates string to list of floats
        try:
            box_coordinates_list = [float(x.strip()) for x in box_coordinates_value.split(',')]
        except ValueError:
            print(f"Warning: Invalid box_coordinates format: {box_coordinates_value}")
            # Use field-based default values
            box_coordinates_list = get_box_coordinates(field_value)
    # Set object positions based on field
    object_mesh_positions = get_object_positions(field_value)
    

    # Get MoveIt configs for dual arm
    moveit_config = (
        MoveItConfigsBuilder("dual_arm", package_name="robot_config")
        .robot_description(
            file_path="config/dual_arm.urdf.xacro",
            mappings={
                "use_fake_hardware": "true",
                # Joint limits for left arm
                "left_Revolute_1_lower_limit": left_Revolute_1_lower_limit,
                "left_Revolute_1_upper_limit": left_Revolute_1_upper_limit,
                "left_Revolute_2_lower_limit": left_Revolute_2_lower_limit,
                "left_Revolute_2_upper_limit": left_Revolute_2_upper_limit,
                "left_Revolute_3_lower_limit": left_Revolute_3_lower_limit,
                "left_Revolute_3_upper_limit": left_Revolute_3_upper_limit,
                "left_Slider_1_lower_limit": left_Slider_1_lower_limit,
                "left_Slider_1_upper_limit": left_Slider_1_upper_limit,
                "left_Slider_2_lower_limit": left_Slider_2_lower_limit,
                "left_Slider_2_upper_limit": left_Slider_2_upper_limit,
                "left_Revolute_4_lower_limit": left_Revolute_4_lower_limit,
                "left_Revolute_4_upper_limit": left_Revolute_4_upper_limit,
                "left_Revolute_6_lower_limit": left_Revolute_6_lower_limit,
                "left_Revolute_6_upper_limit": left_Revolute_6_upper_limit,
                # Joint limits for right arm
                "right_Revolute_1_lower_limit": right_Revolute_1_lower_limit,
                "right_Revolute_1_upper_limit": right_Revolute_1_upper_limit,
                "right_Revolute_2_lower_limit": right_Revolute_2_lower_limit,
                "right_Revolute_2_upper_limit": right_Revolute_2_upper_limit,
                "right_Revolute_3_lower_limit": right_Revolute_3_lower_limit,
                "right_Revolute_3_upper_limit": right_Revolute_3_upper_limit,
                "right_Slider_1_lower_limit": right_Slider_1_lower_limit,
                "right_Slider_1_upper_limit": right_Slider_1_upper_limit,
                "right_Slider_2_lower_limit": right_Slider_2_lower_limit,
                "right_Slider_2_upper_limit": right_Slider_2_upper_limit,
                "right_Revolute_4_lower_limit": right_Revolute_4_lower_limit,
                "right_Revolute_4_upper_limit": right_Revolute_4_upper_limit,
                "right_Revolute_6_lower_limit": right_Revolute_6_lower_limit,
                "right_Revolute_6_upper_limit": right_Revolute_6_upper_limit,
                # Joint origins for left arm
                "left_Revolute_1_xyz": left_Revolute_1_xyz,
                "left_Revolute_1_rpy": left_Revolute_1_rpy,
                "left_Revolute_2_xyz": left_Revolute_2_xyz,
                "left_Revolute_2_rpy": left_Revolute_2_rpy,
                "left_Revolute_3_xyz": left_Revolute_3_xyz,
                "left_Revolute_3_rpy": left_Revolute_3_rpy,
                "left_Revolute_4_xyz": left_Revolute_4_xyz,
                "left_Revolute_4_rpy": left_Revolute_4_rpy,
                "left_Revolute_5_xyz": left_Revolute_5_xyz,
                "left_Revolute_5_rpy": left_Revolute_5_rpy,
                "left_Revolute_6_xyz": left_Revolute_6_xyz,
                "left_Revolute_6_rpy": left_Revolute_6_rpy,
                "left_Rigid_7_xyz": left_Rigid_7_xyz,
                "left_Rigid_7_rpy": left_Rigid_7_rpy,
                "left_Slider_1_xyz": left_Slider_1_xyz,
                "left_Slider_1_rpy": left_Slider_1_rpy,
                "left_Slider_2_xyz": left_Slider_2_xyz,
                "left_Slider_2_rpy": left_Slider_2_rpy,
                # Joint origins for right arm
                "right_Revolute_1_xyz": right_Revolute_1_xyz,
                "right_Revolute_1_rpy": right_Revolute_1_rpy,
                "right_Revolute_2_xyz": right_Revolute_2_xyz,
                "right_Revolute_2_rpy": right_Revolute_2_rpy,
                "right_Revolute_3_xyz": right_Revolute_3_xyz,
                "right_Revolute_3_rpy": right_Revolute_3_rpy,
                "right_Revolute_4_xyz": right_Revolute_4_xyz,
                "right_Revolute_4_rpy": right_Revolute_4_rpy,
                "right_Revolute_5_xyz": right_Revolute_5_xyz,
                "right_Revolute_5_rpy": right_Revolute_5_rpy,
                "right_Revolute_6_xyz": right_Revolute_6_xyz,
                "right_Revolute_6_rpy": right_Revolute_6_rpy,
                "right_Rigid_7_xyz": right_Rigid_7_xyz,
                "right_Rigid_7_rpy": right_Rigid_7_rpy,
                "right_Slider_1_xyz": right_Slider_1_xyz,
                "right_Slider_1_rpy": right_Slider_1_rpy,
                "right_Slider_2_xyz": right_Slider_2_xyz,
                "right_Slider_2_rpy": right_Slider_2_rpy,
                # Arm origins
                "left_origin_xyz": left_origin_xyz,
                "left_origin_rpy": left_origin_rpy,
                "right_origin_xyz": right_origin_xyz,
                "right_origin_rpy": right_origin_rpy,
            },
        )
        .robot_description_semantic(file_path="config/dual_arm.srdf")
        .robot_description_kinematics(file_path="config/dual_arm_kinematics.yaml")
        .joint_limits(file_path="config/dual_arm_joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner","chomp","stomp"],
            default_planning_pipeline="ompl"
        )
        .trajectory_execution(
            file_path="config/dual_arm_moveit_controllers.yaml",
        )
        .to_moveit_configs()
    )
    
    # Explicitly load OMPL configuration from file
    ompl_config_file = os.path.join(
        get_package_share_directory("robot_config"),
        "config",
        "dual_arm_ompl_planning.yaml"
    )
    
    # Load the OMPL configuration
    with open(ompl_config_file, 'r') as file:
        ompl_config = yaml.safe_load(file)
    
    # Update the moveit_config with the loaded OMPL configuration
    moveit_config.planning_pipelines["ompl"].update(ompl_config)

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
    use_dual_controller = LaunchConfiguration('use_dual_controller', default='false')

    spawn_controllers = []
    # Spawn joint_state_broadcaster FIRST for faster availability of joint_states
    spawn_controllers.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen",
        )
    )

    # Conditionally spawn controllers based on use_dual_controller
    # When dual is enabled: spawn dual_arm_controller, hands, seiretu
    # When dual is disabled: spawn left/right arm controllers, hands, seiretu
    # Use conditional Nodes instead of OpaqueFunction for robustness
    spawn_controllers.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dual_arm_controller", "-c", "/controller_manager"],
            condition=IfCondition(use_dual_controller),
            output="screen",
        )
    )
    spawn_controllers.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["left_arm_controller", "-c", "/controller_manager"],
            condition=UnlessCondition(use_dual_controller),
            output="screen",
        )
    )
    spawn_controllers.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["right_arm_controller", "-c", "/controller_manager"],
            condition=UnlessCondition(use_dual_controller),
            output="screen",
        )
    )
    # Hands + seiretu always
    for ctl in ["left_hand_controller", "right_hand_controller", "red_seiretu_controller", "blue_seiretu_controller"]:
        spawn_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[ctl, "-c", "/controller_manager"],
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
            {"planning_scene_monitor_options.default_attached_padd": 0.0},
            {"planning_scene_monitor_options.default_robot_padd": 0.0},
            # Set OMPL random seed for reproducible planning
            {"ompl/random_seed": 42},
            # Explicitly set planning pipeline
            {"default_planning_pipeline": "ompl"},
            # Disable trajectory execution monitoring to prevent repetitive execution
            {"trajectory_execution.execution_duration_monitoring": False},
            {"trajectory_execution.allowed_execution_duration_scaling": 10.0},
            {"trajectory_execution.allowed_goal_duration_margin": 30.0},
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
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        # Explicitly set planning pipeline
        {"default_planning_pipeline": "ompl"},
        # Pass the full planning pipeline configuration
        moveit_config.planning_pipelines,
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
    
    # Static TF for camera
    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "1.5", "0.0", "0.0", "0.0", "world", "camera_link"],
    )

    # Servo nodes for realtime control (configured for both arms)
    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="left_servo_node",
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
                        "linear": 0.4,
                        "rotational": 0.8,
                        "joint": 0.5,
                    },
                    "low_latency_mode": False,
                    "publish_period": 0.034,
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
                    "num_outgoing_halt_msgs_to_publish": 4,
                    "lower_singularity_threshold": 30.0,
                    "hard_stop_singularity_threshold": 90.0,
                    "joint_limit_margin": 0.1,
                    "cartesian_command_in_topic": "/left_servo_node/delta_twist_cmds",
                    "joint_command_in_topic": "/left_servo_node/delta_joint_cmds",
                    "joint_topic": "joint_states",
                    "status_topic": "/left_servo_node/status",
                    "command_out_topic": "/left_arm_controller/joint_trajectory",
                    "check_collisions": True,
                    "collision_check_rate": 10.0,
                    "self_collision_proximity_threshold": 0.01,
                    "scene_collision_proximity_threshold": 0.02,
                }
            },
        ],
        output="screen",
    )

    right_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="right_servo_node",
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
                        "linear": 0.4,
                        "rotational": 0.8,
                        "joint": 0.5,
                    },
                    "low_latency_mode": False,
                    "publish_period": 0.034,
                    "command_out_type": "trajectory_msgs/JointTrajectory",
                    "publish_joint_positions": True,
                    "publish_joint_velocities": False,
                    "publish_joint_accelerations": False,
                    "smoothing_filter_plugin_name": "online_signal_smoothing::ButterworthFilterPlugin",
                    "move_group_name": "right_arm",
                    "planning_frame": "world",
                    "ee_frame_name": "right_EndEffector_1",
                    "robot_link_command_frame": "world",
                    "incoming_command_timeout": 1.0,
                    "num_outgoing_halt_msgs_to_publish": 4,
                    "lower_singularity_threshold": 30.0,
                    "hard_stop_singularity_threshold": 90.0,
                    "joint_limit_margin": 0.1,
                    "cartesian_command_in_topic": "/right_servo_node/delta_twist_cmds",
                    "joint_command_in_topic": "/right_servo_node/delta_joint_cmds",
                    "joint_topic": "joint_states",
                    "status_topic": "/right_servo_node/status",
                    "command_out_topic": "/right_arm_controller/joint_trajectory",
                    "check_collisions": True,
                    "collision_check_rate": 10.0,
                    "self_collision_proximity_threshold": 0.01,
                    "scene_collision_proximity_threshold": 0.02,
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
            {"use_dual_controller": use_dual_controller},
        ],
    )

    # Node to publish collision mesh (optimized for reliable mesh display)
    publish_collision_mesh_node = TimerAction(
        period=5.0,  # 5 second delay to ensure stable initialization
        actions=[
            Node(
                package="robot_config",
                executable="publish_collision_mesh.py",
                name="publish_collision_mesh",
                output="screen",
                parameters=[
                    {"field": field},
                    {"field_mesh_path": "/home/a/ws_moveit2/src/field_description-20250822T021318Z-1-001/field_description/meshes/base_link.stl"},
                    # {"object_mesh_path": "/home/a/ws_moveit2/src/object_description-20250821T110253Z-1-001/object_description/meshes/base_link.stl"},  # COMMENTED OUT
                    # {"object_mesh_positions": object_mesh_positions},  # COMMENTED OUT
                    {"box_coordinates": box_coordinates_list},  # Now uses parsed list of coordinates
                    {"continuous_publishing": True},  # Enable continuous republishing
                    {"republish_interval": 3.0},  # Republish every 3 seconds
                    {"initial_republishes": 5},  # Publish 5 times initially to ensure visibility
                    {"initial_republish_interval": 1.0},  # 1 second between initial publishes
                    {"retry_on_failure": True},  # Retry if publishing fails
                ],
            )
        ]
    )

    # Get box_coordinates value from command line
    box_coordinates_value = "0.134,-0.05528,-0.001,0.134,-0.05528,1.0,0.134,-0.05528,-0.001,0.134,-0.05528,1.0,0.564494,-0.05528,-0.001,0.564494,-0.05528,1.0,0.564494,-0.05528,-0.001,0.564494,-0.05528,1.0"  # Default
    
    # Check command line arguments for both field and box_coordinates parameters
    import sys
    # Node to sort joint states

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

    # ROS TCP Endpoint for Unity
    ros_tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        output="screen",
        parameters=[{"ROS_IP": "0.0.0.0"}],
    )

    # NPM dev server
    npm_run_dev = ExecuteProcess(
        cmd=['npm', 'run', 'dev'],
        cwd='/home/a/ws_moveit2/src/robot_config/catch_gui',
        shell=True,
        output='screen'
    )

    # Pose command publisher node
    pose_command_publisher_node = Node(
        package="robot_config",
        executable="pose_command_publisher",
        name="pose_command_publisher",
        output="screen",
    )

    # Dual Arm Servo Control node (for web app control)
    dual_arm_servo_control_node = Node(
        package="robot_config",
        executable="dual_arm_servo_controller",
        name="dual_arm_servo_controller",
        output="screen",
        parameters=[
            {"velocity_scale": 0.5},
            {"timeout_duration": 1.0},
            {"left_servo_topic": "/left_servo_node/delta_twist_cmds"},
            {"right_servo_topic": "/right_servo_node/delta_twist_cmds"},
        ],
    )

    # Web video server for camera streaming (optimized)
    web_video_server_node = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        parameters=[
            {"port": 8080},
            {"address": "0.0.0.0"},
            {"default_stream_type": "mjpeg"},  # MJPEG for better performance
            {"refresh_rate": 30.0},  # Refresh rate in Hz
            {"quality": 80},  # JPEG compression quality
            {"width": 1280},   # Reduce resolution for better performance
            {"height": 720},  # Reduce resolution for better performance
            {"frame_rate": 15.0}  # Reduce frame rate for better performance
        ],
    )

    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ])
        ]),
        launch_arguments={
            "camera_namespace": "camera",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "pointcloud.enable": "true",
        }.items()
    )

    # D415 RGB depth 3D launch with dynamic camera position based on field
    camera_position_x = PythonExpression([
        '"1.69" if "', field, '" == "red" else ',
        '"0.0" if "', field, '" == "blue" else "0.00"'
    ])
    
    camera_position_y = PythonExpression([
        '"-0.1795" if "', field, '" == "red" else ',
        '"-0.1795" if "', field, '" == "blue" else "-0.1795"'
    ])
    
    camera_position_z = "1.0"  # Z position remains the same
    
    camera_yaw_deg = PythonExpression([
        '"180.0" if "', field, '" == "red" else ',
        '"0.0" if "', field, '" == "blue" else "0.0"'
    ])
    
    d415_rgb_depth_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("d415_rgb_depth_3d"),
                "launch",
                "d415_rgb_depth_3d.launch.py"
            ])
        ]),
        launch_arguments={
            "camera_position_x": camera_position_x,
            "camera_position_y": camera_position_y,
            "camera_position_z": camera_position_z,
            "camera_yaw_deg": camera_yaw_deg,
        }.items()
    )

    # Dynamixel controller node
    dynamixel_controller_node = Node(
        package="dynamixel_controller",
        executable="dynamixel_controller_node",
        name="dynamixel_controller_node",
        output="screen",
        parameters=["/home/a/ws_moveit2/src/dynamixel_ros2/dynamixel_controller/config/bus_config.yaml"],
    )

    # Dynamixel GUI node
    dynamixel_gui_node = Node(
        package="dynamixel_controller_gui",
        executable="dynamixel_gui",
        name="dynamixel_gui",
        output="screen",
    )
    

    target_pose_router = Node(
        package='robot_config',
        executable='target_pose_router.py',
        name='target_pose_router',
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
            left_servo_node,
            right_servo_node,
            joy_node,
            move_to_pose_dual_cpp_node, # Add the new dual arm node
            # Seiretu robots are now integrated into main robot_description
            # No separate publishers needed
            # Additional nodes
            rosbridge_websocket,
            rosapi_node,
            ros_tcp_endpoint,  # Add the ROS TCP Endpoint for Unity
            npm_run_dev,
            pose_command_publisher_node,  # Add pose command publisher node
            dual_arm_servo_control_node,  # Add dual arm servo control node
            web_video_server_node,  # Add web video server for camera streaming
            realsense_launch,  # Add RealSense camera launch
            d415_rgb_depth_3d_launch,  # Add D415 RGB depth 3D launch
            camera_static_tf,  # Add camera static transform
            publish_collision_mesh_node,
            dynamixel_controller_node,  # Add Dynamixel controller
            dynamixel_gui_node,  # Add Dynamixel GUI
            target_pose_router,  # Add target pose router
        ]
    )
