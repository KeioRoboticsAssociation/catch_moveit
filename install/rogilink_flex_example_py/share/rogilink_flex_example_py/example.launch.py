import os
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 引数（必要に応じて上書きできる）
    config_default = '/home/aa/ros2_ws/src/rogilinkFlex-ros2/rogilink_flex/config/config.json'
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=config_default),
        # 任意：/dev/serial/by-id/... を渡すと確実にそのポートに接続
        DeclareLaunchArgument('port0', default_value=''),  # 例: /dev/serial/by-id/usb-...dev0...
        DeclareLaunchArgument('port1', default_value=''),  # 例: /dev/serial/by-id/usb-...dev1...

        # === device 0 用 rogilink_flex ===
        Node(
            package='rogilink_flex',
            executable='rogilink_flex',
            name='rogilink_flex_0',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'select_device_index': 0,                 # ← devices[0] を使う
                'port': LaunchConfiguration('port0'),     # 空なら自動スキャン
            }]
        ),

        # === device 1 用 rogilink_flex ===
        Node(
            package='rogilink_flex',
            executable='rogilink_flex',
            name='rogilink_flex_1',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'select_device_index': 1,                 # ← devices[1] を使う
                'port': LaunchConfiguration('port1'),
            }]
        ),

        # === 両方を扱う example ノード（あなたのPython） ===
        Node(
            package='rogilink_flex_example_py',
            executable='example_node',
            name='example_node'
        ),
    ])


if __name__ == "__main__":
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
