import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from stm32_mavlink_interface.msg import RobomasterMotorCommand

class MAVLinkControllerClient(Node):
    def __init__(self):
        super().__init__('mavlink_controller_client')

        # joint_states データを保存する変数
        self.left_revolute_1_pos = 0.0
        self.right_revolute_1_pos = 0.0

        # RoboMaster Motor制御用パブリッシャー
        self.motor_cmd_publisher = self.create_publisher(
            RobomasterMotorCommand,
            'robomaster/motor_command',
            100
        )

        # joint_states サブスクライバー
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            100
        )

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
        self.send_motor_command(5, self.left_revolute_1_pos)

        # ID6: right_Revolute_1
        self.send_motor_command(6, self.right_revolute_1_pos)

    def send_motor_command(self, motor_id, target_position):
        """モーターに制御コマンドを送信"""
        cmd_msg = RobomasterMotorCommand()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.motor_id = motor_id
        cmd_msg.control_mode = cmd_msg.CONTROL_MODE_POSITION
        cmd_msg.target_position_rad = target_position
        cmd_msg.enabled = True

        self.motor_cmd_publisher.publish(cmd_msg)

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