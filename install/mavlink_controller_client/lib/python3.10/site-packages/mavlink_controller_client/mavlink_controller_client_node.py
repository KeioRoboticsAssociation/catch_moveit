import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from stm32_mavlink_interface.msg import RobomasterMotorCommand, RobomasterMotorConfig
from rogilink_flex_lib import Publisher, Subscriber
from ctypes import c_float
from std_msgs.msg import Float32MultiArray

class MAVLinkControllerClient(Node):
    def __init__(self):
        super().__init__('mavlink_controller_client')

        # joint_states データを保存する変数
        self.left_revolute_1_pos = 0.0
        self.right_revolute_1_pos = 0.0

        # # RoboMaster Motor制御用パブリッシャー
        # self.motor_cmd_publisher = self.create_publisher(
        #     RobomasterMotorCommand,
        #     'robomaster/motor_command',
        #     100
        # )

        # # RoboMaster Motor設定用パブリッシャー
        # self.motor_config_publisher = self.create_publisher(
        #     RobomasterMotorConfig,
        #     'robomaster/motor_config',
        #     100
        # )

        # joint_states サブスクライバー
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            100
        )

        # # PIDコマンド用サブスクライバー
        # self.pid_command_subscription = self.create_subscription(
        #     String,
        #     'pid_command',
        #     self.pid_command_callback,
        #     10
        # )

        self.pub_yaw = Publisher(self, 'YAW' ,(c_float, c_float))

        # 定期的にモーター制御コマンドを送信（100Hz）
        self.motor_control_timer = self.create_timer(0.01, self.motor_control_callback)

        self.get_logger().info('MAVLink Controller Client initialized')

    def joint_states_callback(self, msg):
        """joint_states データを保存（positionのみ）"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                if name == "left_Revolute_1":
                    self.left_revolute_1_pos = msg.position[i]
                elif name == "right_Revolute_1":
                    self.right_revolute_1_pos = msg.position[i]

    def motor_control_callback(self):
        """モーター制御コマンドを送信"""
        # ID5: left_Revolute_1
        # self.send_motor_command(5, self.left_revolute_1_pos)

        # # ID6: right_Revolute_1
        # self.send_motor_command(6, self.right_revolute_1_pos)
        # self.get_logger().info(f'Left Revolute 1 Position: {self.left_revolute_1_pos}, Right Revolute 1 Position: {self.right_revolute_1_pos}')
        self.pub_yaw.publish((-self.left_revolute_1_pos, self.right_revolute_1_pos))

    # def send_motor_command(self, motor_id, target_position):
    #     """モーターに制御コマンドを送信"""
    #     # cmd_msg = RobomasterMotorCommand()
    #     # cmd_msg.header.stamp = self.get_clock().now().to_msg()
    #     # cmd_msg.motor_id = motor_id
    #     # cmd_msg.control_mode = cmd_msg.CONTROL_MODE_POSITION
    #     # cmd_msg.target_position_rad = target_position
    #     # cmd_msg.enabled = True

    #     # self.motor_cmd_publisher.publish(cmd_msg)

    # def pid_command_callback(self, msg):
    #     """PIDコマンドを処理してMAVLinkメッセージID 182で送信"""
    #     try:
    #         # Format: "motor_id,param_name,value"
    #         # Example: "5,VEL_KP,50.0" or "5,POS_KP,25.0"
    #         parts = msg.data.split(',')
    #         if len(parts) != 3:
    #             self.get_logger().error(f"Invalid PID command format: {msg.data}")
    #             self.get_logger().info("Expected format: 'motor_id,param_name,value'")
    #             self.get_logger().info("Example: '5,VEL_KP,50.0'")
    #             return

    #         motor_id = int(parts[0])
    #         param_name = parts[1].strip()
    #         value = float(parts[2])

    #         # 有効なパラメータ名をチェック
    #         valid_params = ['POS_KP', 'POS_KI', 'POS_KD', 'VEL_KP', 'VEL_KI', 'VEL_KD',
    #                       'MAX_VEL', 'MAX_CUR', 'MAX_TEMP', 'TIMEOUT']

    #         if param_name not in valid_params:
    #             self.get_logger().error(f"Invalid parameter name: {param_name}")
    #             self.get_logger().info(f"Valid parameters: {valid_params}")
    #             return

    #         if motor_id < 1 or motor_id > 8:
    #             self.get_logger().error(f"Invalid motor ID: {motor_id} (must be 1-8)")
    #             return

    #         # MAVLinkパラメータ名を生成 (例: "M5_VEL_KP")
    #         mavlink_param_name = f"M{motor_id}_{param_name}"

    #         self.get_logger().info(f"Setting {mavlink_param_name} = {value}")

    #         # RobomasterMotorConfigメッセージで送信
    #         self.send_motor_config(motor_id, param_name, value)

    #     except ValueError as e:
    #         self.get_logger().error(f"Error parsing PID command: {e}")
    #         self.get_logger().info("Example: '5,VEL_KP,50.0'")
    #     except Exception as e:
    #         self.get_logger().error(f"Unexpected error in PID command: {e}")

    # def send_motor_config(self, motor_id, param_name, value):
    #     """モーターにPID設定を送信"""
    #     config_msg = RobomasterMotorConfig()
    #     config_msg.header.stamp = self.get_clock().now().to_msg()
    #     config_msg.motor_id = motor_id

    #     # パラメータに応じて対応するフィールドを設定
    #     if param_name == 'POS_KP':
    #         config_msg.position_kp = value
    #     elif param_name == 'POS_KI':
    #         config_msg.position_ki = value
    #     elif param_name == 'POS_KD':
    #         config_msg.position_kd = value
    #     elif param_name == 'VEL_KP':
    #         config_msg.velocity_kp = value
    #     elif param_name == 'VEL_KI':
    #         config_msg.velocity_ki = value
    #     elif param_name == 'VEL_KD':
    #         config_msg.velocity_kd = value
    #     elif param_name == 'MAX_VEL':
    #         config_msg.max_velocity_rps = value
    #     elif param_name == 'MAX_CUR':
    #         config_msg.max_current_ma = int(value)
    #     elif param_name == 'MAX_TEMP':
    #         config_msg.max_temperature_celsius = int(value)
    #     elif param_name == 'TIMEOUT':
    #         config_msg.watchdog_timeout_ms = int(value)

    #     self.motor_config_publisher.publish(config_msg)
    #     self.get_logger().info(f"Published motor config for Motor {motor_id}: {param_name} = {value}")

def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkControllerClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()