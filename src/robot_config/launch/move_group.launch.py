from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("0525_arm", package_name="robot_config")\
        .planning_pipelines(pipelines=["ompl", "chomp", "stomp", "pilz_industrial_motion_planner"])\
        .to_moveit_configs()
    
    return generate_move_group_launch(moveit_config)
