import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- パスとファイル名の設定 ---
    pkg_name = 'catch_gazebo_control' # ★あなたのパッケージ名に変更
    robot_name_in_model = '0525_arm'
    urdf_file_name = '0525_arm.urdf'
    world_file_name = 'basic.world'
    
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # --- robot_description の設定 ---
    with open(urdf_model_path, 'r') as f:
        robot_description_content = f.read()
    robot_description = {'robot_description': robot_description_content}

    # --- Gazeboの起動 ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # --- ノードの定義 ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name_in_model],
        output='screen',
    )
    
    # --- コントローラーの起動 (Spawnerノードを使用) ---
    controller_names = [
        'joint_state_broadcaster',
        'rtop_controller',
        'rmid_controller',
        'rbtm_controller',
        'ltop_controller',
        'lmid_controller',
        'lbtm_controller',
        'joint_trajectory_controller', # 必要ならコメントを外す
    ]
    
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
        ) for name in controller_names
    ]

    # --- 実行シーケンスの定義 ---
    # Gazeboにロボットがスポーンされた後、コントローラー群を起動する
    spawn_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=controller_spawners,
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        spawn_controllers_event,
    ])
