#!/usr/bin/env python3
"""
カメラレイ可視化のテストスクリプト
RVizでカメラ位置と光線を表示する独立したノード
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point as PixelPoint
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

class RayVisualizerTest(Node):
    def __init__(self):
        super().__init__('ray_visualizer_test')
        
        # パブリッシャー
        self.pub_markers = self.create_publisher(MarkerArray, '/d415/test_ray_markers', 10)
        
        # パラメータ
        self.declare_parameter('camera_x', 0.0)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.80)
        self.declare_parameter('target_x', 0.5)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.10)
        
        camera_x = self.get_parameter('camera_x').value
        camera_y = self.get_parameter('camera_y').value
        camera_z = self.get_parameter('camera_z').value
        target_x = self.get_parameter('target_x').value
        target_y = self.get_parameter('target_y').value
        target_z = self.get_parameter('target_z').value
        
        self.camera_pos = np.array([camera_x, camera_y, camera_z])
        self.target_pos = np.array([target_x, target_y, target_z])
        
        # タイマーで定期的にマーカーを発行
        self.timer = self.create_timer(1.0, self.publish_test_markers)
        
        self.get_logger().info(f'Ray visualizer test started')
        self.get_logger().info(f'Camera: {self.camera_pos}')
        self.get_logger().info(f'Target: {self.target_pos}')
    
    def publish_test_markers(self):
        """テスト用のレイマーカーを発行"""
        marker_array = MarkerArray()
        frame_id = "world"
        stamp = self.get_clock().now().to_msg()
        
        # 1. カメラレイ（矢印）
        ray_marker = Marker()
        ray_marker.header.frame_id = frame_id
        ray_marker.header.stamp = stamp
        ray_marker.ns = "test_ray"
        ray_marker.id = 0
        ray_marker.type = Marker.ARROW
        ray_marker.action = Marker.ADD
        
        # 始点と終点
        start_point = PixelPoint()
        start_point.x, start_point.y, start_point.z = float(self.camera_pos[0]), float(self.camera_pos[1]), float(self.camera_pos[2])
        end_point = PixelPoint()
        end_point.x, end_point.y, end_point.z = float(self.target_pos[0]), float(self.target_pos[1]), float(self.target_pos[2])
        
        ray_marker.points = [start_point, end_point]
        
        # スタイル
        ray_marker.scale.x = 0.02  # 軸の太さ
        ray_marker.scale.y = 0.04  # 頭の太さ
        ray_marker.scale.z = 0.05  # 頭の長さ
        
        # 色（赤）
        ray_marker.color.r = 1.0
        ray_marker.color.g = 0.0
        ray_marker.color.b = 0.0
        ray_marker.color.a = 0.9
        
        marker_array.markers.append(ray_marker)
        
        # 2. カメラ位置（球体、青）
        camera_marker = Marker()
        camera_marker.header.frame_id = frame_id
        camera_marker.header.stamp = stamp
        camera_marker.ns = "test_camera"
        camera_marker.id = 1
        camera_marker.type = Marker.SPHERE
        camera_marker.action = Marker.ADD
        
        camera_marker.pose.position.x = float(self.camera_pos[0])
        camera_marker.pose.position.y = float(self.camera_pos[1])
        camera_marker.pose.position.z = float(self.camera_pos[2])
        camera_marker.pose.orientation.w = 1.0
        
        camera_marker.scale.x = 0.08
        camera_marker.scale.y = 0.08
        camera_marker.scale.z = 0.08
        
        camera_marker.color.r = 0.0
        camera_marker.color.g = 0.0
        camera_marker.color.b = 1.0
        camera_marker.color.a = 1.0
        
        marker_array.markers.append(camera_marker)
        
        # 3. 目標位置（立方体、緑）
        target_marker = Marker()
        target_marker.header.frame_id = frame_id
        target_marker.header.stamp = stamp
        target_marker.ns = "test_target"
        target_marker.id = 2
        target_marker.type = Marker.CUBE
        target_marker.action = Marker.ADD
        
        target_marker.pose.position.x = float(self.target_pos[0])
        target_marker.pose.position.y = float(self.target_pos[1])
        target_marker.pose.position.z = float(self.target_pos[2])
        target_marker.pose.orientation.w = 1.0
        
        target_marker.scale.x = 0.05
        target_marker.scale.y = 0.05
        target_marker.scale.z = 0.05
        
        target_marker.color.r = 0.0
        target_marker.color.g = 1.0
        target_marker.color.b = 0.0
        target_marker.color.a = 1.0
        
        marker_array.markers.append(target_marker)
        
        # 4. Z平面（薄い黄色の板）
        plane_marker = Marker()
        plane_marker.header.frame_id = frame_id
        plane_marker.header.stamp = stamp
        plane_marker.ns = "test_plane"
        plane_marker.id = 3
        plane_marker.type = Marker.CUBE
        plane_marker.action = Marker.ADD
        
        plane_marker.pose.position.x = 0.0
        plane_marker.pose.position.y = 0.0
        plane_marker.pose.position.z = float(self.target_pos[2])  # 目標のZ座標
        plane_marker.pose.orientation.w = 1.0
        
        plane_marker.scale.x = 3.0
        plane_marker.scale.y = 3.0
        plane_marker.scale.z = 0.002
        
        plane_marker.color.r = 1.0
        plane_marker.color.g = 1.0
        plane_marker.color.b = 0.0
        plane_marker.color.a = 0.2
        
        marker_array.markers.append(plane_marker)
        
        # 5. 座標軸（原点）
        for i, (axis, color) in enumerate([('X', [1,0,0]), ('Y', [0,1,0]), ('Z', [0,0,1])]):
            axis_marker = Marker()
            axis_marker.header.frame_id = frame_id
            axis_marker.header.stamp = stamp
            axis_marker.ns = f"test_axis_{axis.lower()}"
            axis_marker.id = 4 + i
            axis_marker.type = Marker.ARROW
            axis_marker.action = Marker.ADD
            
            origin = PixelPoint()
            origin.x, origin.y, origin.z = 0.0, 0.0, 0.0
            
            end = PixelPoint()
            if i == 0:  # X軸
                end.x, end.y, end.z = 0.3, 0.0, 0.0
            elif i == 1:  # Y軸
                end.x, end.y, end.z = 0.0, 0.3, 0.0
            else:  # Z軸
                end.x, end.y, end.z = 0.0, 0.0, 0.3
            
            axis_marker.points = [origin, end]
            
            axis_marker.scale.x = 0.01
            axis_marker.scale.y = 0.02
            axis_marker.scale.z = 0.03
            
            axis_marker.color.r = color[0]
            axis_marker.color.g = color[1]
            axis_marker.color.b = color[2]
            axis_marker.color.a = 0.8
            
            marker_array.markers.append(axis_marker)
        
        # マーカー配列を発行
        self.pub_markers.publish(marker_array)

def main():
    rclpy.init()
    node = RayVisualizerTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:Axis
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
