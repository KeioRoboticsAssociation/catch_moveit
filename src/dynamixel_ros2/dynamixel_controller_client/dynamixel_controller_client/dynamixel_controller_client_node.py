import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
from dynamixel_controller.msg import DynamixelController, DynamixelResponse, DynamixelCommand  # C++ノードで定義したメッセージをインポート

class DynamixelControllerClient(Node):
    ids = [1,2,3,4,5,6]
    def __init__(self):
        super().__init__('dynamixel_controller_client')
        
        # joint_states データを保存する変数
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_efforts = {}
        self.joint_names = []
        
        
        # 命令送信用トピック
        self.tx_publisher = self.create_publisher(DynamixelCommand, 'dynamixel_tx', 100)
        # 初期化時に全モーターをリブート後、トルクオン
        # self.reboot_all_motors()
        # self.enable_torque()
        
        # C++ノードからの応答受信用トピック
        self.rx_subscription = self.create_subscription(
            DynamixelResponse,
            'dynamixel_rx',
            self.rx_callback,
            100
        )
        
        # joint_states サブスクライバー
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            100
        )
        
        # 5秒ごとに SYNC_WRITE 命令, 1秒ごとに SYNC_READ 命令を送信するタイマー
        self.write_timer = self.create_timer(0.01, self.write_callback)
        self.read_timer = self.create_timer(1.0, self.read_callback)
        
    def write_callback(self):
        # SYNC_WRITE テスト: 複数モーターに目標位置を送信
        msg = DynamixelCommand()
        msg.command = DynamixelController.SYNC_WRITE
        msg.address = 116  # GOAL_POSITION アドレス
        msg.length = 4     # 4バイト
        msg.ids = self.ids # 全てのモーター

        # 各モーターの目標位置
        target_positions = {
            1: 2048 - int(self.joint_positions.get("left_Revolute_2", 0.0) * 2048 / 3.14),   # joint1 (TTL,XM540)
            2: 3072 - int(self.joint_positions.get("left_Revolute_3", 0.0) * 2048 / 3.14),   # joint2 (TTL, XM540)
            3: 3072 + int(self.joint_positions.get("left_Revolute_4", 0.0) * 2048 / 3.14),   # joint3 (RS485, XL430)
            4: int(self.joint_positions.get("left_Revolute_5", 0.0) * 2048 / 3.14),   # joint4 (RS485, XL430)
            5: 3072 + int(self.joint_positions.get("left_Revolute_6", 0.0) * 2048 / 3.14),   # joint5 (RS485, XL430)
            6: 57 - int(self.joint_positions.get("left_Slider_1", 0.0) / 0.024 * 910),   # joint6 (RS485, XL430)
        }
        
        # 各モーターの位置データを4バイトずつ結合（Extended Position Control用）
        msg.data = []
        for motor_id in msg.ids:
            position = target_positions[motor_id]
            # Extended Position Control Modeでは符号付き32ビット整数
            position_bytes = position.to_bytes(4, 'little', signed=True)
            msg.data.extend(position_bytes)
        
        self.tx_publisher.publish(msg)
        # self.get_logger().info(f'Published SYNC_WRITE: IDs={msg.ids}, Positions={[target_positions[id] for id in msg.ids]}')
        
    def read_callback(self):
        # SYNC_READ 命令
        msg = DynamixelCommand()
        msg.command = DynamixelController.SYNC_READ
        msg.address = 132   # PRESENT_POSITION アドレス
        msg.length = 4      # 4バイト
        msg.ids = [1, 2, 3, 4, 5, 6]
        msg.data = []       # SYNC_READでは空
        
        self.tx_publisher.publish(msg)
        # self.get_logger().info(f'Published SYNC_READ command: IDs={msg.ids}, Address={msg.address}, Length={msg.length}')
        
    def rx_callback(self, msg):
        self.get_logger().info(f'Received response: IDs={msg.ids}, Values={msg.data}')
    
    def joint_states_callback(self, msg):
        # joint_states データを保存
        self.joint_names = msg.name
        # 辞書形式で保存（ジョイント名をキーに）
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
                # self.get_logger().info(f'{name}: {self.joint_positions[name]}')
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.joint_efforts[name] = msg.effort[i]
    
    def get_joint_position(self, joint_name):
        """特定のジョイントの位置を取得"""
        return self.joint_positions.get(joint_name, 0.0)
    
    def get_joint_velocity(self, joint_name):
        """特定のジョイントの速度を取得"""
        return self.joint_velocities.get(joint_name, 0.0)
    
    def get_all_positions(self):
        """全てのジョイント位置を取得"""
        return self.joint_positions.copy()

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControllerClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()