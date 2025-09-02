#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import threading
import time

class RealtimeArmControl(Node):
    def __init__(self):
        super().__init__('realtime_arm_control')
        
        # サブスクライバーを作成（左右のアーム用）
        self.left_arm_sub = self.create_subscription(
            Twist,
            '/left_arm_realtime_control',
            self.left_arm_callback,
            10
        )
        
        self.right_arm_sub = self.create_subscription(
            Twist,
            '/right_arm_realtime_control',
            self.right_arm_callback,
            10
        )
        
        # パブリッシャーを作成（左右のアーム用）
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            10
        )
        
        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )
        
        # ジョイントステートサブスクライバー
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # サービスクライアントを作成
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        
        # ジョイント状態を保持
        self.current_joint_states = {}
        
        # 制御パラメータ
        self.control_rate = 50.0  # Hz
        self.control_period = 1.0 / self.control_rate
        
        # ジョイント名
        self.left_arm_joints = [
            'left_Revolute_1', 'left_Revolute_2', 'left_Revolute_3',
            'left_Revolute_4', 'left_Revolute_5', 'left_Revolute_6'
        ]
        
        self.right_arm_joints = [
            'right_Revolute_1', 'right_Revolute_2', 'right_Revolute_3',
            'right_Revolute_4', 'right_Revolute_5', 'right_Revolute_6'
        ]
        
        self.get_logger().info('Realtime Arm Control node has been started.')
        self.get_logger().info('Subscribing to /left_arm_realtime_control and /right_arm_realtime_control')

    def joint_state_callback(self, msg):
        \"\"\"ジョイント状態のコールバック\"\"\"
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]

    def left_arm_callback(self, msg):
        \"\"\"左アーム制御コールバック\"\"\"
        self.control_arm(msg, 'left')

    def right_arm_callback(self, msg):
        \"\"\"右アーム制御コールバック\"\"\"
        self.control_arm(msg, 'right')

    def control_arm(self, twist_msg, arm_side):
        \"\"\"アーム制御のメイン処理\"\"\"
        # 現在のエンドエフェクタ位置を取得
        if arm_side == 'left':
            ee_link = 'left_EndEffector_1'
            joints = self.left_arm_joints
            publisher = self.left_arm_pub
        else:
            ee_link = 'right_EndEffector_1'
            joints = self.right_arm_joints
            publisher = self.right_arm_pub
        
        # ジョイント状態が取得できているか確認
        if not all(joint in self.current_joint_states for joint in joints):
            self.get_logger().warn(f'{arm_side} arm joint states not available yet.')
            return
        
        # Twistメッセージから制御指令を抽出
        linear_x = twist_msg.linear.x
        linear_y = twist_msg.linear.y
        linear_z = twist_msg.linear.z
        angular_x = twist_msg.angular.x
        angular_y = twist_msg.angular.y
        angular_z = twist_msg.angular.z
        
        # ジョイント位置を更新
        # ここでは簡略化のため、直接ジョイント位置を変更
        # 実際のアプリケーションでは、より複雑な運動学計算が必要
        joint_positions = [self.current_joint_states[joint] for joint in joints]
        
        # 簡単な位置更新（実際には運動学計算が必要）
        # ここではデモとして小さな変化を加える
        if abs(linear_x) > 0.01 or abs(linear_y) > 0.01 or abs(linear_z) > 0.01 or \
           abs(angular_x) > 0.01 or abs(angular_y) > 0.01 or abs(angular_z) > 0.01:
            
            # 各軸の変化量をジョイントに適用（簡略化）
            scale_factor = 0.01  # スケーリングファクター
            joint_positions[0] += angular_z * scale_factor  # ヨー回転
            joint_positions[1] += linear_z * scale_factor   # 垂直移動
            joint_positions[2] += linear_x * scale_factor   # 前後移動
            joint_positions[3] += linear_y * scale_factor   # 左右移動
            
            # ジョイント制限を考慮（簡略化）
            for i in range(len(joint_positions)):
                joint_positions[i] = max(-3.14, min(3.14, joint_positions[i]))
            
            # JointTrajectoryメッセージを作成
            traj_msg = JointTrajectory()
            traj_msg.joint_names = joints
            traj_msg.header = Header()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = Duration(sec=0, nanosec=int(self.control_period * 1e9))
            
            traj_msg.points.append(point)
            
            # トピックにパブリッシュ
            publisher.publish(traj_msg)
            
            self.get_logger().debug(f'{arm_side} arm control: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), '
                                   f'angular=({angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeArmControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()