import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
from dynamixel_controller.msg import DynamixelController, DynamixelResponse, DynamixelCommand  # C++ノードで定義したメッセージをインポート

class DynamixelControllerClient(Node):
    ids = [0,1,2,3,4,5,6]
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
    
    # def reboot_all_motors(self):
    #     # 全モーターをリブートする（フィードバック制御付き）
    #     self.get_logger().info('Rebooting all motors with feedback control...')
        
    #     rebooted_ids = set()
    #     online_ids = set()
    #     max_attempts = 3000  # 100Hz * 30秒（十分な時間）
    #     attempt = 0
        
    #     # 一時的にコールバックを設定
    #     self.reboot_check_response = None
    #     self.reboot_check_subscription = self.create_subscription(
    #         DynamixelResponse,
    #         'dynamixel_rx',
    #         self.reboot_check_callback,
    #         100
    #     )
        
    #     while len(online_ids) < len(self.ids) and attempt < max_attempts:
    #         # まだリブートしていないモーターにREBOOTコマンド送信
    #         for motor_id in self.ids:
    #             if motor_id not in rebooted_ids:
    #                 msg = DynamixelCommand()
    #                 msg.command = DynamixelController.REBOOT
    #                 msg.ids = [motor_id]
    #                 msg.address = 0
    #                 msg.length = 0
    #                 msg.data = []
    #                 self.tx_publisher.publish(msg)
    #                 rebooted_ids.add(motor_id)
                    
    #         # 100Hz待機 (10ms)
    #         time.sleep(0.01)
            
    #         # PINGで全モーターの生存確認
    #         ping_msg = DynamixelCommand()
    #         ping_msg.command = DynamixelController.PING
    #         ping_msg.ids = self.ids
    #         ping_msg.address = 0
    #         ping_msg.length = 0
    #         ping_msg.data = []
            
    #         self.reboot_check_response = None
    #         self.tx_publisher.publish(ping_msg)
            
    #         # レスポンスを待つ (5ms以内)
    #         timeout = 0.005
    #         start_time = time.time()
    #         while self.reboot_check_response is None and (time.time() - start_time) < timeout:
    #             rclpy.spin_once(self, timeout_sec=0.001)
            
    #         # レスポンスを処理
    #         if self.reboot_check_response is not None:
    #             for i, motor_id in enumerate(self.reboot_check_response.ids):
    #                 if i < len(self.reboot_check_response.error) and self.reboot_check_response.error[i] == 0:
    #                     online_ids.add(motor_id)
                
    #             if attempt % 100 == 0:  # 1秒ごとにログ出力
    #                 self.get_logger().info(f'Attempt {attempt + 1}: Motors online: {sorted(online_ids)} / {self.ids}')
            
    #         attempt += 1
            
    #         # 残り5ms待機して100Hz維持
    #         time.sleep(0.005)
        
    #     # 一時的なサブスクリプションを削除
    #     self.destroy_subscription(self.reboot_check_subscription)
        
    #     if len(online_ids) == len(self.ids):
    #         self.get_logger().info(f'Successfully rebooted and verified all motors in {attempt} attempts ({attempt*0.01:.2f}s): {sorted(online_ids)}')
    #     else:
    #         self.get_logger().warn(f'Failed to verify all motors after {max_attempts} attempts ({max_attempts*0.01:.2f}s). '
    #                              f'Online: {sorted(online_ids)}, Target: {self.ids}')
    
    # def enable_torque(self):
    #     # 複数モーターのトルクを有効にする（100Hz SYNC_READで確認しながら）
        
    #     torque_enabled_ids = set()
    #     max_attempts = 1000  # 100Hz * 10秒
    #     attempt = 0
        
    #     # 一時的にコールバックを設定
    #     self.torque_check_response = None
    #     self.torque_check_subscription = self.create_subscription(
    #         DynamixelResponse,
    #         'dynamixel_rx',
    #         self.torque_check_callback,
    #         100
    #     )
        
    #     while len(torque_enabled_ids) < len(self.ids) and attempt < max_attempts:
    #         # SYNC_WRITEでトルク有効化
    #         msg = DynamixelCommand()
    #         msg.command = DynamixelController.SYNC_WRITE
    #         msg.address = 64   # TORQUE_ENABLE アドレス
    #         msg.length = 1     # 1バイト
    #         msg.ids = self.ids # 全てのモーター
    #         msg.data = [1] * len(self.ids)  # 各モーターに対して1=トルクON
            
    #         self.tx_publisher.publish(msg)
            
    #         # 100Hz待機 (10ms)
    #         time.sleep(0.01)
            
    #         # SYNC_READでトルク状態を確認
    #         read_msg = DynamixelCommand()
    #         read_msg.command = DynamixelController.SYNC_READ
    #         read_msg.address = 64   # TORQUE_ENABLE アドレス
    #         read_msg.length = 1     # 1バイト
    #         read_msg.ids = self.ids
    #         read_msg.data = []
            
    #         self.torque_check_response = None
    #         self.tx_publisher.publish(read_msg)
            
    #         # レスポンスを待つ (5ms以内)
    #         timeout = 0.005
    #         start_time = time.time()
    #         while self.torque_check_response is None and (time.time() - start_time) < timeout:
    #             rclpy.spin_once(self, timeout_sec=0.001)
            
    #         # レスポンスを処理
    #         if self.torque_check_response is not None:
    #             for i, motor_id in enumerate(self.torque_check_response.ids):
    #                 if i < len(self.torque_check_response.data) and self.torque_check_response.data[i] == 1:
    #                     torque_enabled_ids.add(motor_id)
                
    #             if attempt % 100 == 0:  # 1秒ごとにログ出力
    #                 self.get_logger().info(f'Attempt {attempt + 1}: Torque enabled for IDs: {sorted(torque_enabled_ids)} / {self.ids}')
            
    #         attempt += 1
            
    #         # 残り5ms待機して100Hz維持
    #         time.sleep(0.005)
        
    #     # 一時的なサブスクリプションを削除
    #     self.destroy_subscription(self.torque_check_subscription)
        
    #     if len(torque_enabled_ids) == len(self.ids):
    #         self.get_logger().info(f'Successfully enabled torque for all motors in {attempt} attempts ({attempt*0.01:.2f}s): {sorted(torque_enabled_ids)}')
    #     else:
    #         self.get_logger().warn(f'Failed to enable torque for some motors after {max_attempts} attempts ({max_attempts*0.01:.2f}s). '
    #                              f'Enabled: {sorted(torque_enabled_ids)}, Target: {self.ids}')
    
    # def reboot_check_callback(self, msg):
    #     # リブート確認用の一時的なコールバック（PING応答を処理）
    #     if msg.command == DynamixelController.PING:
    #         self.reboot_check_response = msg
            
    # def torque_check_callback(self, msg):
    #     # トルク確認用の一時的なコールバック
    #     if msg.command == DynamixelController.SYNC_READ:
    #         self.torque_check_response = msgdef reboot_all_motors(self):
    #     # 全モーターをリブートする（フィードバック制御付き）
    #     self.get_logger().info('Rebooting all motors with feedback control...')
        
    #     rebooted_ids = set()
    #     online_ids = set()
    #     max_attempts = 3000  # 100Hz * 30秒（十分な時間）
    #     attempt = 0
        
    #     # 一時的にコールバックを設定
    #     self.reboot_check_response = None
    #     self.reboot_check_subscription = self.create_subscription(
    #         DynamixelResponse,
    #         'dynamixel_rx',
    #         self.reboot_check_callback,
    #         100
    #     )
        
    #     while len(online_ids) < len(self.ids) and attempt < max_attempts:
    #         # まだリブートしていないモーターにREBOOTコマンド送信
    #         for motor_id in self.ids:
    #             if motor_id not in rebooted_ids:
    #                 msg = DynamixelCommand()
    #                 msg.command = DynamixelController.REBOOT
    #                 msg.ids = [motor_id]
    #                 msg.address = 0
    #                 msg.length = 0
    #                 msg.data = []
    #                 self.tx_publisher.publish(msg)
    #                 rebooted_ids.add(motor_id)
                    
    #         # 100Hz待機 (10ms)
    #         time.sleep(0.01)
            
    #         # PINGで全モーターの生存確認
    #         ping_msg = DynamixelCommand()
    #         ping_msg.command = DynamixelController.PING
    #         ping_msg.ids = self.ids
    #         ping_msg.address = 0
    #         ping_msg.length = 0
    #         ping_msg.data = []
            
    #         self.reboot_check_response = None
    #         self.tx_publisher.publish(ping_msg)
            
    #         # レスポンスを待つ (5ms以内)
    #         timeout = 0.005
    #         start_time = time.time()
    #         while self.reboot_check_response is None and (time.time() - start_time) < timeout:
    #             rclpy.spin_once(self, timeout_sec=0.001)
            
    #         # レスポンスを処理
    #         if self.reboot_check_response is not None:
    #             for i, motor_id in enumerate(self.reboot_check_response.ids):
    #                 if i < len(self.reboot_check_response.error) and self.reboot_check_response.error[i] == 0:
    #                     online_ids.add(motor_id)
                
    #             if attempt % 100 == 0:  # 1秒ごとにログ出力
    #                 self.get_logger().info(f'Attempt {attempt + 1}: Motors online: {sorted(online_ids)} / {self.ids}')
            
    #         attempt += 1
            
    #         # 残り5ms待機して100Hz維持
    #         time.sleep(0.005)
        
    #     # 一時的なサブスクリプションを削除
    #     self.destroy_subscription(self.reboot_check_subscription)
        
    #     if len(online_ids) == len(self.ids):
    #         self.get_logger().info(f'Successfully rebooted and verified all motors in {attempt} attempts ({attempt*0.01:.2f}s): {sorted(online_ids)}')
    #     else:
    #         self.get_logger().warn(f'Failed to verify all motors after {max_attempts} attempts ({max_attempts*0.01:.2f}s). '
    #                              f'Online: {sorted(online_ids)}, Target: {self.ids}')
    
    # def enable_torque(self):
    #     # 複数モーターのトルクを有効にする（100Hz SYNC_READで確認しながら）
        
    #     torque_enabled_ids = set()
    #     max_attempts = 1000  # 100Hz * 10秒
    #     attempt = 0
        
    #     # 一時的にコールバックを設定
    #     self.torque_check_response = None
    #     self.torque_check_subscription = self.create_subscription(
    #         DynamixelResponse,
    #         'dynamixel_rx',
    #         self.torque_check_callback,
    #         100
    #     )
        
    #     while len(torque_enabled_ids) < len(self.ids) and attempt < max_attempts:
    #         # SYNC_WRITEでトルク有効化
    #         msg = DynamixelCommand()
    #         msg.command = DynamixelController.SYNC_WRITE
    #         msg.address = 64   # TORQUE_ENABLE アドレス
    #         msg.length = 1     # 1バイト
    #         msg.ids = self.ids # 全てのモーター
    #         msg.data = [1] * len(self.ids)  # 各モーターに対して1=トルクON
            
    #         self.tx_publisher.publish(msg)
            
    #         # 100Hz待機 (10ms)
    #         time.sleep(0.01)
            
    #         # SYNC_READでトルク状態を確認
    #         read_msg = DynamixelCommand()
    #         read_msg.command = DynamixelController.SYNC_READ
    #         read_msg.address = 64   # TORQUE_ENABLE アドレス
    #         read_msg.length = 1     # 1バイト
    #         read_msg.ids = self.ids
    #         read_msg.data = []
            
    #         self.torque_check_response = None
    #         self.tx_publisher.publish(read_msg)
            
    #         # レスポンスを待つ (5ms以内)
    #         timeout = 0.005
    #         start_time = time.time()
    #         while self.torque_check_response is None and (time.time() - start_time) < timeout:
    #             rclpy.spin_once(self, timeout_sec=0.001)
            
    #         # レスポンスを処理
    #         if self.torque_check_response is not None:
    #             for i, motor_id in enumerate(self.torque_check_response.ids):
    #                 if i < len(self.torque_check_response.data) and self.torque_check_response.data[i] == 1:
    #                     torque_enabled_ids.add(motor_id)
                
    #             if attempt % 100 == 0:  # 1秒ごとにログ出力
    #                 self.get_logger().info(f'Attempt {attempt + 1}: Torque enabled for IDs: {sorted(torque_enabled_ids)} / {self.ids}')
            
    #         attempt += 1
            
    #         # 残り5ms待機して100Hz維持
    #         time.sleep(0.005)
        
    #     # 一時的なサブスクリプションを削除
    #     self.destroy_subscription(self.torque_check_subscription)
        
    #     if len(torque_enabled_ids) == len(self.ids):
    #         self.get_logger().info(f'Successfully enabled torque for all motors in {attempt} attempts ({attempt*0.01:.2f}s): {sorted(torque_enabled_ids)}')
    #     else:
    #         self.get_logger().warn(f'Failed to enable torque for some motors after {max_attempts} attempts ({max_attempts*0.01:.2f}s). '
    #                              f'Enabled: {sorted(torque_enabled_ids)}, Target: {self.ids}')
    
    # def reboot_check_callback(self, msg):
    #     # リブート確認用の一時的なコールバック（PING応答を処理）
    #     if msg.command == DynamixelController.PING:
    #         self.reboot_check_response = msg
            
    # def torque_check_callback(self, msg):
    #     # トルク確認用の一時的なコールバック
    #     if msg.command == DynamixelController.SYNC_READ:
    #         self.torque_check_response = msg
        
    def write_callback(self):
        # SYNC_WRITE テスト: 複数モーターに目標位置を送信
        msg = DynamixelCommand()
        msg.command = DynamixelController.SYNC_WRITE
        msg.address = 116  # GOAL_POSITION アドレス
        msg.length = 4     # 4バイト
        msg.ids = self.ids # 全てのモーター

        # 各モーターの目標位置
        target_positions = {
            0: 3584 + int(self.joint_positions.get("left_Revolute_1", 0.0) * 2048 / 3.14),   # joint0 (TTL,XM540)
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
        # if 'right_Revolute_1' in self.joint_positions:
        #     pos = self.joint_positions['right_Revolute_1']
        #     vel = self.joint_velocities.get('right_Revolute_1', 0.0)
        #     self.get_logger().info(f'right_Revolute_1: pos={pos:.3f}, vel={vel:.3f}')
        # 辞書形式で保存（ジョイント名をキーに）
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
                # self.get_logger().info(f'{name}: {self.joint_positions[name]}')
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.joint_efforts[name] = msg.effort[i]
        
        # 特定のジョイントの値を使う例
        # if 'right_Revolute_1' in self.joint_positions:
        #     pos = self.joint_positions['right_Revolute_1']
        #     vel = self.joint_velocities.get('right_Revolute_1', 0.0)
        #     self.get_logger().info(f'right_Revolute_1: pos={pos:.3f}, vel={vel:.3f}')
    
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