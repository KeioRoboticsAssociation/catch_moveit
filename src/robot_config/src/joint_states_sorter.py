#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

class JointStatesSorter(Node):
    def __init__(self):
        super().__init__('joint_states_sorter')
        
        # 整列したい関節名のリスト（任意の順序）
        self.sorted_joint_names = [
            'left_Revolute_1',
            'left_Revolute_2',
            'left_Revolute_3',
            'left_Revolute_4',
            'left_Revolute_5',
            'left_Revolute_6',
            'right_Revolute_1',
            'right_Revolute_2',
            'right_Revolute_3',
            'right_Revolute_4',
            'right_Revolute_5',
            'right_Revolute_6',
            'left_Slider_1',
            'left_Slider_2',
            'right_Slider_1',
            'right_Slider_2',
        ]
        
        # /joint_statesトピックを受信するサブスクライバ
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        
        # 整列された関節位置をパブリッシュするパブリッシャ
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/sorted_joint_positions', 
            10)
                
        self.get_logger().info('JointStatesSorter node started.')

    def joint_states_callback(self, msg):
        # 関節名と位置のマッピングを作成
        joint_positions = dict(zip(msg.name, msg.position))
        
        # 指定された順序で関節位置を取得
        sorted_positions = []
        for joint_name in self.sorted_joint_names:
            if joint_name in joint_positions:
                position = joint_positions[joint_name]
                sorted_positions.append(float(position))
            else:
                # 関節が見つからない場合は0.0を追加
                sorted_positions.append(0.0)
        
        # Float64MultiArrayメッセージを作成
        position_msg = Float64MultiArray()
        
        # layoutフィールドを設定
        layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = "joint_positions"
        dim.size = len(sorted_positions)
        dim.stride = len(sorted_positions)
        layout.dim = [dim]
        layout.data_offset = 0
        
        # 各関節名をコメントとして追加（実際のメッセージには含まれませんが、
        # デバッグのためにログに出力）
        layout.dim[0].label = ",".join(self.sorted_joint_names)
        
        position_msg.layout = layout
        position_msg.data = sorted_positions
        
        # パブリッシュ
        self.publisher_.publish(position_msg)
        
        # ログ出力（デバッグ用）
        # self.get_logger().info(f'Published sorted positions: {sorted_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatesSorter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()