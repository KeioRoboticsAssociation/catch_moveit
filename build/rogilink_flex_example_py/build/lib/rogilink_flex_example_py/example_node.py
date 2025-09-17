import time
from ctypes import *

import rclpy
from rclpy.node import Node
from rogilink_flex_lib import Publisher, Subscriber
from catch_interfaces.msg import F446ToROS, ROSToF446


class RogiLinkFlexExampleNode(Node):
    def __init__(self):
        super().__init__('RogiLinkFlexExampleNode')
        self.get_logger().info("RogiLinkFlexExampleNode started")
        self.create_publisher(F446ToROS, 'f446_to_ros', 10)
        self.create_subscription(ROSToF446, 'ros_to_f446', self.ros_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        # self.pub0 = Publisher(self, 'a', c_float, device_id=0)
        # self.sub0 = Subscriber(self, 'a', c_float, self.sub_callback0, device_id=0)
        # self.pub1 = Publisher(self, 'b', c_float, device_id=1)
        # self.sub1 = Subscriber(self, 'b', c_float, self.sub_callback1, device_id=1)
        self.a = 1.0
        self.b = 2.0
        
    def timer_callback(self):
        # self.pub0.publish(self.a)
        # self.pub1.publish(self.b)
        pass

    # def sub_callback0(self, value):
    #     self.get_logger().info(f"Received on 'a': {value}")

    # def sub_callback1(self, value):
    #     self.get_logger().info(f"Received on 'b': {value}")
        
    def ros_callback(self, msg):
        self.get_logger().info(f"Received on 'ros_to_f446': {msg.data}")

def main():
    rclpy.init()
    node = RogiLinkFlexExampleNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
        