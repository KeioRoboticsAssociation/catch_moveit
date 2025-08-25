import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
from dynamixel_controller.msg import DynamixelController, DynamixelResponse, DynamixelCommand  # C++ノードで定義したメッセージをインポート

class DynamixelControllerClient(Node):
    ids = [1, 2, 3, 4, 5]
    def __init__(self):
        super().__init__('dynamixel_controller_client')
        
        # 命令送信用トピック
        self.tx_publisher = self.create_publisher(DynamixelCommand, 'dynamixel_tx', 10)

        # msg = UInt8MultiArray()
        
        # msg.data = [DynamixelController.SYNC_WRITE, DynamixelController.OPERATING_MODE, 1]
        # for dxl_id in self.ids:
        #     msg.data.extend([dxl_id, 3])  # Position Controlモード3
        # self.tx_publisher.publish(msg)
        
        # msg.data = [DynamixelController.SYNC_WRITE, DynamixelController.TORQUE_ENABLE, 1]
        # for dxl_id in self.ids:
        #     msg.data.extend([dxl_id, 1])  # トルクを有効にする
        # self.tx_publisher.publish(msg)
        
        # C++ノードからの応答受信用トピック
        self.rx_subscription = self.create_subscription(
            DynamixelResponse,
            'dynamixel_rx',
            self.rx_callback,
            10
        )
        
        # joint_states サブスクライバー
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10
        )
        # 初期化時にトルクオン
        self.enable_torque()
        
        # 5秒ごとに SYNC_WRITE 命令, 1秒ごとに SYNC_READ 命令を送信するタイマー
        self.write_timer = self.create_timer(5.0, self.write_callback)
        self.read_timer = self.create_timer(1.0, self.read_callback)
    
    def enable_torque(self):
        # 複数モーターのトルクを有効にする
        msg = DynamixelCommand()
        msg.command = DynamixelController.SYNC_WRITE
        msg.address = 64   # TORQUE_ENABLE アドレス
        msg.length = 1     # 1バイト
        msg.ids = self.ids # 全てのモーター
        msg.data = [1] * len(self.ids)  # 各モーターに対して1=トルクON
        
        self.tx_publisher.publish(msg)
        self.get_logger().info(f'Torque enabled for IDs={msg.ids}')
        
    def write_callback(self):
        # SYNC_WRITE テスト: 複数モーターに目標位置を送信
        msg = DynamixelCommand()
        msg.command = DynamixelController.SYNC_WRITE
        msg.address = 116  # GOAL_POSITION アドレス
        msg.length = 4     # 4バイト
        msg.ids = self.ids # 全てのモーター

        # 各モーターの目標位置
        target_positions = {
            1: 2048,   # joint1 (TTL,XM540)
            2: 3072,  # joint2 (TTL, XM540) 
            3: 3072,  # joint3 (RS485, XL430)
            4: 4096,  # joint4 (RS485, XL430)
            5: 3072,   # joint5 (RS485, XL430)
            # 6: 800,   # joint6 (RS485, XL430)
        }
        
        # 各モーターの位置データを4バイトずつ結合
        msg.data = []
        for motor_id in msg.ids:
            position = target_positions[motor_id]
            position_bytes = position.to_bytes(4, 'little')
            msg.data.extend(position_bytes)
        
        self.tx_publisher.publish(msg)
        self.get_logger().info(f'Published SYNC_WRITE: IDs={msg.ids}, Positions={[target_positions[id] for id in msg.ids]}')
        
    def read_callback(self):
        # SYNC_READ 命令
        msg = DynamixelCommand()
        msg.command = DynamixelController.SYNC_READ
        msg.address = 132   # PRESENT_POSITION アドレス
        msg.length = 4      # 4バイト
        msg.ids = [1, 2, 3, 4, 5, 6]
        msg.data = []       # SYNC_READでは空
        
        self.tx_publisher.publish(msg)
        self.get_logger().info(f'Published SYNC_READ command: IDs={msg.ids}, Address={msg.address}, Length={msg.length}')
        
    def rx_callback(self, msg):
        self.get_logger().info(f'Received response: IDs={msg.ids}, Values={msg.data}')
    
    def joint_states_callback(self, msg):
        # joint_states メッセージを受信したときの処理
        self.get_logger().info(f'Received joint_states: {len(msg.name)} joints')
        
        # 各ジョイントの情報を表示（簡略化）
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else 0.0
            velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.get_logger().info(f'  {name}: pos={position:.3f}, vel={velocity:.3f}')

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
