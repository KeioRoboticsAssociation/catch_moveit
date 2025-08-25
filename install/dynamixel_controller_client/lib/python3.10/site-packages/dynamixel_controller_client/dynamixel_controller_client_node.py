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
        # 5秒ごとに SYNC_WRITE 命令, 1秒ごとに SYNC_READ 命令を送信するタイマー
        # self.write_timer = self.create_timer(5.0, self.write_callback)
        self.read_timer = self.create_timer(1.0, self.read_callback)
        
    # def write_callback(self):
    #     # SYNC_WRITE 命令のフォーマット: [SYNC_WRITE, goal_address, data_length, id(1), joint_position(1)]
    #     msg = UInt8MultiArray()
    #     SYNC_WRITE = 131   # DynamixelController.msg で定義した SYNC_WRITE (0x83)
    #     goal_address = 116  # 目標位置アドレス
    #     data_length = 4
        
    #     def encode_goal_position(dxl_id, position):
    #     # 位置を4バイトのリトルエンディアンに変換
    #         pos_bytes = position.to_bytes(4, 'little')
    #         return [dxl_id] + list(pos_bytes)

    #     joint_positions = {
    #         1: 512, # joint1 (TTL,XM540)
    #         2: 400, # joint2 (TTL, XM540)
    #         3: 300, # joint3 (RS485, XL430)
    #         4: 600, # joint4 (RS485, XL430)
    #         5: 200, # joint5 (RS485, XL430)
    #         6: 128, # joint6 (RS485, XL430)
    #         7: 900  # joint7 (RS485, XL430)
    #     }

    #     msg.data = [SYNC_WRITE, goal_address, data_length]
    #     for dxl_id in self.ids:
    #         msg.data += encode_goal_position(dxl_id, joint_positions[dxl_id])
    #     self.tx_publisher.publish(msg)
    #     self.get_logger().info(f'Publisher SYNC_WRITE command: {msg.data}')
        
    def read_callback(self):
        # SYNC_READ 命令
        msg = DynamixelCommand()
        msg.command = DynamixelController.SYNC_READ
        msg.address = 132   # PRESENT_POSITION アドレス
        msg.length = 4      # 4バイト
        msg.ids = [3, 4, 5, 6]
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
