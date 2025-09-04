#!/usr/bin/env python3
import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from geometry_msgs.msg import Point as PixelPoint
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from typing import Optional

# Coordinate conventions:
#   Camera (RealSense): +X right, +Y down, +Z forward
#   World (right-handed): +X forward, +Y left, +Z up

# ---------- math utils ----------
def R_to_quat(R: np.ndarray) -> Quaternion:
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))

# RollPitchYaw(deg) -> 回転行列
def euler_deg_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0],[sy, cy, 0],[0,0,1]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz @ Ry @ Rx  # ZYX

def quat_to_R_np(x, y, z, w):
    q = np.array([x, y, z, w], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=np.float64)


# ---------- node ----------
class D415TapToPose(Node):
    """
    入力:
      - /camera/color/image_raw (Image) ※任意
      - /camera/aligned_depth_to_color/image_raw (Image) ※depthモードのみ必要
      - /camera/color/camera_info (CameraInfo)
      - /tap/d415_pixel (geometry_msgs/Point) ・・・ x=u, y=v

    出力:
      - /d415/target_pose (PoseStamped)
    """
    def __init__(self):
        super().__init__('d415_rgb_depth_3d_node')
        self.bridge = CvBridge()

        # ---- Parameters ----
        # topics
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('tap_topic', '/tap/d415_pixel')

        # modes & misc
        self.declare_parameter('projection_mode', 'fixed_world_z')  # 'fixed_world_z' | 'depth'
        self.declare_parameter('median_ksize', 5)
        self.declare_parameter('target_frame', 'world')  # 空なら camera or world で自動

        # fixed world Z (table height)
        self.declare_parameter('fixed_world_z', 0.10)  # [m] world Z height

        # depth mode only
        self.declare_parameter('hover_offset', 0.03)   # [m] upward safety offset before descent

        # fixed extrinsics (world <- camera)
        # 固定カメラ位置
        self.declare_parameter('camera_position_x', 0.0)
        self.declare_parameter('camera_position_y', 0.0)
        self.declare_parameter('camera_position_z', 0.80)
        # 姿勢（どちらか一方を使う）
        self.declare_parameter('orientation_mode', 'rpy')  # 'rpy' | 'quat'
        self.declare_parameter('camera_orientation_x', 0.0)
        self.declare_parameter('camera_orientation_y', 0.0)
        self.declare_parameter('camera_orientation_z', 0.0)
        self.declare_parameter('camera_orientation_w', 0.0)
        self.declare_parameter('camera_roll_deg', 0.0)
        self.declare_parameter('camera_pitch_deg', 30.0)
        self.declare_parameter('camera_yaw_deg', 0.0)

        # read params
        self.color_topic  = self.get_parameter('color_topic').value
        self.depth_topic  = self.get_parameter('depth_topic').value
        self.info_topic   = self.get_parameter('camera_info_topic').value
        self.tap_topic    = self.get_parameter('tap_topic').value
        self.projection_mode = self.get_parameter('projection_mode').value
        self.median_ksize = int(self.get_parameter('median_ksize').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.fixed_world_z = float(self.get_parameter('fixed_world_z').value) 
        self.hover_offset = float(self.get_parameter('hover_offset').value) 

        # fixed extrinsics (directly in new world coordinates: +X forward, +Y left, +Z up)
        tx = float(self.get_parameter('camera_position_x').value)
        ty = float(self.get_parameter('camera_position_y').value)
        tz = float(self.get_parameter('camera_position_z').value)
        self.t_wc = np.array([tx, ty, tz], dtype=np.float64)

        # 姿勢の決定
        self.orientation_mode = self.get_parameter('orientation_mode').value
        # 置き換え（姿勢の決定部）
        if self.orientation_mode == 'quat':
            qx = float(self.get_parameter('camera_orientation_x').value)
            qy = float(self.get_parameter('camera_orientation_y').value)
            qz = float(self.get_parameter('camera_orientation_z').value)
            qw = float(self.get_parameter('camera_orientation_w').value)
            self.R_wc = quat_to_R_np(qx, qy, qz, qw)
        else:  # 'rpy'
            roll  = float(self.get_parameter('camera_roll_deg').value)
            pitch = float(self.get_parameter('camera_pitch_deg').value)
            yaw   = float(self.get_parameter('camera_yaw_deg').value)
            self.R_wc = euler_deg_to_R(roll, pitch, yaw)

        # subscriptions
        self.sub_color = self.create_subscription(Image, self.color_topic, self.on_color, 10)
        if self.projection_mode == 'depth':
            self.sub_depth = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.sub_info  = self.create_subscription(CameraInfo, self.info_topic, self.on_info, 10)
        self.sub_tap   = self.create_subscription(PixelPoint, self.tap_topic, self.on_tap, 10)

        # publisher
        self.pub_target = self.create_publisher(PoseStamped, '/d415/target_pose', 10)
        self.pub_ray_marker = self.create_publisher(Marker, '/d415/ray_visualization', 10)

        # buffers
        self.depth_img: Optional[np.ndarray] = None  # used only in depth mode
        self.K = None  # (fx, fy, cx, cy)

        self.get_logger().info(f'D415TapToPose ready. mode={self.projection_mode}')

    # ---------- callbacks ----------
    def on_color(self, msg: Image):
        # 使わないが、同期・デバッグ用に保持したい場合はここで処理
        pass

    def on_depth(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if img.dtype == np.uint16:
            self.depth_img = img.astype(np.float32) / 1000.0
        else:
            self.depth_img = img.astype(np.float32)

    def on_info(self, msg: CameraInfo):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy

    # ---------- core helpers ----------
    def deproject_with_depth(self, u: int, v: int) -> Optional[np.ndarray]:
        """深度を使って（x,y,z）[camera]を計算"""
        if self.depth_img is None or self.K is None:
            return None
        h, w = self.depth_img.shape[:2]
        u = int(np.clip(u, 0, w-1)); v = int(np.clip(v, 0, h-1))
        
        # タップ点近傍の中央値でノイズを抑える
        k = max(1, self.median_ksize | 1)
        half = k // 2
        v0, v1 = max(0, v-half), min(h, v+half+1)
        u0, u1 = max(0, u-half), min(w, u+half+1)
        z = float(np.median(self.depth_img[v0:v1, u0:u1]))
        if not np.isfinite(z) or z <= 0.0:
            return None
        
        fx, fy, cx, cy = self.K
        # Apply coordinate transformation: +u -> -y, +v -> -x in camera coordinates
        x = (v - cy) / fy * z   # +v -> +x
        y = -(u - cx) / fx * z  # +u -> -y
        return np.array([x, y, z], dtype=np.float64)

    def ray_intersect_world_z(self, u: int, v: int, fixed_world_z: float) -> Optional[np.ndarray]:
        if self.K is None or self.R_wc is None or self.t_wc is None:
            return None
        fx, fy, cx, cy = self.K
        # Apply coordinate transformation: +u -> -y, +v -> -x in world coordinates  
        d_c = np.array([(v - cy) / fy, -(u - cx) / fx, 1.0], dtype=np.float64)
        n = np.linalg.norm(d_c)
        if n < 1e-9:
            return None
        d_c /= n
        d_w = self.R_wc @ d_c
        o_w = self.t_wc
        self.get_logger().warn(
            f"[dbg] o_w={o_w}, d_w={d_w}, "
            f"fixed_z={fixed_world_z:.3f}, "
            f"dz={d_w[2]:.6f}, oz={o_w[2]:.6f}, "
            f"lam={(fixed_world_z - o_w[2]) / (d_w[2] if abs(d_w[2])>1e-12 else np.nan):.6f}"
        )
        if abs(d_w[2]) < 1e-8:
            return None  # レイが水平すぎて交点が求められない
        lam = (fixed_world_z - o_w[2]) / d_w[2]
        if lam <= 0:
            return None  # カメラより手前になる場合は無効
        return o_w + lam * d_w

    # ---------- publish ----------
    def publish_target(self, p_w: np.ndarray, q_w: Quaternion, frame_id: str):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(p_w[0]), float(p_w[1]), float(p_w[2])
        pose.orientation = q_w
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=frame_id)
        self.pub_target.publish(PoseStamped(header=header, pose=pose))

    def publish_ray_visualization(self, camera_pos: np.ndarray, target_pos: np.ndarray, frame_id: str):
        marker = Marker()
        marker.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=frame_id)
        marker.ns = "d415_ray"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow from camera to target
        marker.points = []
        start_point = Point()
        start_point.x, start_point.y, start_point.z = float(camera_pos[0]), float(camera_pos[1]), float(camera_pos[2])
        marker.points.append(start_point)
        
        end_point = Point()
        end_point.x, end_point.y, end_point.z = float(target_pos[0]), float(target_pos[1]), float(target_pos[2])
        marker.points.append(end_point)
        
        # Arrow appearance
        marker.scale.x = 0.01  # shaft diameter
        marker.scale.y = 0.02  # head diameter
        marker.scale.z = 0.03  # head length
        
        # Color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 5  # Display for 5 seconds
        
        self.pub_ray_marker.publish(marker)

    # ---------- main tap ----------
    def on_tap(self, msg: PixelPoint):
        if self.K is None:
            self.get_logger().warn('Waiting for CameraInfo (K).')
            return
        u, v = int(msg.x), int(msg.y)

        if self.projection_mode == 'depth':
            if self.depth_img is None:
                self.get_logger().warn('Waiting for depth image.')
                return
            p_c = self.deproject_with_depth(u, v)
            if p_c is None:
                self.get_logger().warn('Depth invalid at tapped pixel.')
                return
            # camera -> world
            p_w = self.R_wc @ p_c + self.t_wc
            p_w[2] += self.hover_offset  # safety hover (depth mode only)
            q_w = R_to_quat(self.R_wc)
            frame = self.target_frame or 'world'
        else:  # 'fixed_world_z'
            p_w = self.ray_intersect_world_z(u, v, self.fixed_world_z)
            if p_w is None:
                self.get_logger().warn('Failed to intersect with world Z plane (check camera extrinsics).')
                return
            q_w = R_to_quat(self.R_wc)
            frame = self.target_frame or 'world'

        self.publish_target(p_w, q_w, frame)
        self.publish_ray_visualization(self.t_wc, p_w, frame)
        self.get_logger().info(
            f'Published /d415/target_pose ({self.projection_mode}) tap=({u},{v}) -> {p_w} in {frame}.'
        )

# ---------- entry ----------
def main():
    rclpy.init()
    node = D415TapToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()