import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from dynamixel_controller.msg import DynamixelController, DynamixelResponse, DynamixelCommand  # C++ノードで定義したメッセージをインポート

class DynamixelControllerClient(Node):
    ids = [1, 2, 3, 4, 5, 6, 7]
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
        # 初期化時にトルクオン
        # self.enable_torque()
        
        # 5秒ごとに SYNC_WRITE 命令, 1秒ごとに SYNC_READ 命令を送信するタイマー
        # self.write_timer = self.create_timer(5.0, self.write_callback)
        self.read_timer = self.create_timer(1.0, self.read_callback)
    
    # def enable_torque(self):
    #     # ID=3のモーターのトルクを有効にする
    #     msg = DynamixelCommand()
    #     msg.command = DynamixelController.SYNC_WRITE
    #     msg.address = 64   # TORQUE_ENABLE アドレス
    #     msg.length = 1     # 1バイト
    #     msg.ids = [3]      # ID=3のモーター
    #     msg.data = [1]     # 1=トルクON
        
    #     self.tx_publisher.publish(msg)
    #     self.get_logger().info(f'Torque enabled for ID={msg.ids[0]}')
        
    # def write_callback(self):
    #     # SYNC_WRITE テスト: モーターID=3 に目標位置を送信
    #     msg = DynamixelCommand()
    #     msg.command = DynamixelController.SYNC_WRITE
    #     msg.address = 116  # GOAL_POSITION アドレス
    #     msg.length = 4     # 4バイト
    #     msg.ids = [3]      # テスト対象: ID=3のモーター1つ
        
    #     # 目標位置: 512 (中央位置)
    #     target_position = 512
    #     position_bytes = target_position.to_bytes(4, 'little')
    #     msg.data = list(position_bytes)
        
    #     self.tx_publisher.publish(msg)
    #     self.get_logger().info(f'Published SYNC_WRITE: ID={msg.ids[0]}, Position={target_position}')
        
    def read_callback(self):
        # SYNC_READ 命令
        msg = DynamixelCommand()
        msg.command = DynamixelController.SYNC_READ
        msg.address = 132   # PRESENT_POSITION アドレス
        msg.length = 4      # 4バイト
        msg.ids = [1, 3, 4, 5, 6]
        msg.data = []       # SYNC_READでは空
        
        self.tx_publisher.publish(msg)
        self.get_logger().info(f'Published SYNC_READ command: IDs={msg.ids}, Address={msg.address}, Length={msg.length}')
        
    def rx_callback(self, msg):
        self.get_logger().info(f'Received response: IDs={msg.ids}, Values={msg.data}')

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
