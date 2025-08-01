import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # あなたのURDFファイルへのパスを指定
    # 例: 'my_robot_description' パッケージの 'urdf/my_robot.urdf'
    urdf_file_name = '0525_arm.urdf'  # ← ここを修正
    urdf_path = os.path.join(
        get_package_share_directory('0525_arm_description'), # ← ここを修正
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # robot_state_publisher: URDFと/joint_statesから/tfを生成
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # joint_state_publisher_gui: 関節を動かすためのGUIスライダー
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz: 3Dモデルを表示
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('robot_config'), 'config', 'moveit.rviz')] # ← RViz設定ファイルへのパス
        )
    ])