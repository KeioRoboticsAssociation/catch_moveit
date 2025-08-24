#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, RobotCommander
import sys
import time


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # MoveItの初期化
        self.robot = RobotCommander()
        self.left_arm = MoveGroupCommander("left_arm")
        self.right_arm = MoveGroupCommander("right_arm")
        
        # 初期姿勢を設定
        self.set_initial_poses()
        
        self.get_logger().info('初期姿勢を設定しました')
    
    def set_initial_poses(self):
        # left_armの初期姿勢を設定
        left_joint_values = {
            "left_Revolute_1": 0.0,
            "left_Revolute_2": 0.0,
            "left_Revolute_3": 0.0,
            "left_Revolute_4": 0.0,
            "left_Revolute_5": 0.0,
            "left_Revolute_6": 0.0
        }
        
        # right_armの初期姿勢を設定
        right_joint_values = {
            "right_Revolute_1": 0.0,
            "right_Revolute_2": 0.0,
            "right_Revolute_3": 0.0,
            "right_Revolute_4": 0.0,
            "right_Revolute_5": 0.0,
            "right_Revolute_6": 0.0
        }
        
        # 引数から初期姿勢を取得
        for arg in sys.argv[1:]:
            if '=' in arg:
                joint_name, position = arg.split('=')
                if joint_name in left_joint_values:
                    left_joint_values[joint_name] = float(position)
                elif joint_name in right_joint_values:
                    right_joint_values[joint_name] = float(position)
        
        # left_armの姿勢を設定
        self.left_arm.set_joint_value_target(left_joint_values)
        self.left_arm.go(wait=True)
        
        # right_armの姿勢を設定
        self.right_arm.set_joint_value_target(right_joint_values)
        self.right_arm.go(wait=True)
        
        # プランニングシーンを更新
        self.left_arm.stop()
        self.right_arm.stop()


def main(args=None):
    rclpy.init(args=args)
    
    initial_pose_setter = InitialPoseSetter()
    
    # スピンしてサービスを待機
    rclpy.spin_once(initial_pose_setter, timeout_sec=1)
    
    initial_pose_setter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()