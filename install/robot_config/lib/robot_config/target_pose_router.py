#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64MultiArray
import math
from typing import Optional

def quaternion_to_euler(x, y, z, w):
    """クォータニオンをオイラー角(roll, pitch, yaw)に変換"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

class TargetPoseRouter(Node):
    """
    /d415/target_poseを受け取り、/tap/d415_pixelのx座標に応じて
    left_target_pose_rpy または right_target_pose_rpy にルーティングするノード
    
    x <= 639: left_target_pose_rpy
    x >= 640: right_target_pose_rpy
    """
    
    def __init__(self):
        super().__init__('target_pose_router')
        
        # Subscriptions
        self.sub_target_pose = self.create_subscription(
            PoseStamped, '/d415/target_pose', self.on_target_pose, 10)
        self.sub_pixel = self.create_subscription(
            Point, '/tap/d415_pixel', self.on_pixel, 10)
        
        # Publishers
        self.pub_left = self.create_publisher(Float64MultiArray, 'left_target_pose_rpy', 10)
        self.pub_right = self.create_publisher(Float64MultiArray, 'right_target_pose_rpy', 10)
        
        # State
        self.latest_pixel_x: Optional[float] = None
        self.latest_target_pose: Optional[PoseStamped] = None
        
        self.get_logger().info('TargetPoseRouter node started')
    
    def on_pixel(self, msg: Point):
        """ピクセル座標を受信"""
        self.latest_pixel_x = msg.x
        self.get_logger().info(f'Received pixel x: {msg.x}')
        
        # target_poseが既に受信済みの場合は即座にルーティング
        if self.latest_target_pose is not None:
            self.route_target_pose()
    
    def on_target_pose(self, msg: PoseStamped):
        """target_poseを受信"""
        self.latest_target_pose = msg
        self.get_logger().info('Received target_pose')
        
        # ピクセル座標が既に受信済みの場合はルーティング
        if self.latest_pixel_x is not None:
            self.route_target_pose()
    
    def route_target_pose(self):
        """ピクセル座標に基づいてtarget_poseをルーティング"""
        if self.latest_pixel_x is None or self.latest_target_pose is None:
            return
        
        # PoseStampedからFloat64MultiArrayに変換 [x, y, z, roll, pitch, yaw]
        pose = self.latest_target_pose.pose
        
        # ロール、ピッチ、ヨーは全て0に設定
        roll, pitch, yaw = 0.0, 0.0, 0.0
        
        # Float64MultiArrayメッセージを作成
        msg = Float64MultiArray()
        msg.data = [
            pose.position.x,
            pose.position.y, 
            pose.position.z,
            roll,
            pitch,
            yaw
        ]
        
        if self.latest_pixel_x <= 639:
            self.pub_left.publish(msg)
            self.get_logger().info(f'Routed to left_target_pose_rpy (x={self.latest_pixel_x}) - xyz:[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}] rpy:[{roll:.3f}, {pitch:.3f}, {yaw:.3f}]')
        else:  
            self.pub_right.publish(msg)
            self.get_logger().info(f'Routed to right_target_pose_rpy (x={self.latest_pixel_x}) - xyz:[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}] rpy:[{roll:.3f}, {pitch:.3f}, {yaw:.3f}]')
        
        # 使用後はリセット（次回のために）
        self.latest_target_pose = None

def main():
    rclpy.init()
    node = TargetPoseRouter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()