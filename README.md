# これはrvizにキャチアームを表示して動作計画を体験できるものです
moveitの環境構築ができたら0525_arm_description-20250707T055749Z-1-001とrobot_configをクローンして、

"""
cd your_workspace_name
colcon build --packages-select 0525_arm_description-20250707T055749Z-1-001 robot_config
source install/setup.bash
ros2 launch robot_config demo.launch.py
"""

rviz上にアームが出てきてplanとexecuteができたら成功
