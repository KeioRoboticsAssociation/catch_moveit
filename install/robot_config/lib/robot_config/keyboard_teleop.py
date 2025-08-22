#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys, select, tty, termios
import os

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Check if stdin is a terminal (TTY)
        if not os.isatty(sys.stdin.fileno()):
            self.get_logger().error("This node requires a TTY terminal to read keyboard input. Please run it in a terminal.")
            sys.exit(1)
        
        self.joint_positions = {
            'left_Slider_1': 0.0,
            'right_Slider_1': 0.0
        }
        
        self.increment = 0.05
        
        self.joint_state_pub = self.create_publisher(JointState, 'set_joint_trajectory', 10)
        
        # Set up terminal for non-blocking input
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Keyboard Teleop Node has started.')
        self.get_logger().info('Controls:')
        self.get_logger().info('Up Arrow: Increase left_Slider_1')
        self.get_logger().info('Down Arrow: Decrease left_Slider_1')
        self.get_logger().info('Left Arrow: Decrease right_Slider_1')
        self.get_logger().info('Right Arrow: Increase right_Slider_1')
        self.get_logger().info('Press Ctrl+C to exit.')

    def timer_callback(self):
        # Check for keyboard input
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            # Arrow keys send 3 bytes: \x1b[A, \x1b[B, \x1b[C, \x1b[D
            if key == '\x1b':  # ESC character, start of an arrow key sequence
                key += sys.stdin.read(2)
                if key == '\x1b[A':  # Up arrow
                    self.joint_positions['left_Slider_1'] += self.increment
                    self.get_logger().info(f'left_Slider_1 increased to {self.joint_positions["left_Slider_1"]:.2f}')
                elif key == '\x1b[B':  # Down arrow
                    self.joint_positions['left_Slider_1'] -= self.increment
                    self.get_logger().info(f'left_Slider_1 decreased to {self.joint_positions["left_Slider_1"]:.2f}')
                elif key == '\x1b[D':  # Left arrow
                    self.joint_positions['right_Slider_1'] -= self.increment
                    self.get_logger().info(f'right_Slider_1 decreased to {self.joint_positions["right_Slider_1"]:.2f}')
                elif key == '\x1b[C':  # Right arrow
                    self.joint_positions['right_Slider_1'] += self.increment
                    self.get_logger().info(f'right_Slider_1 increased to {self.joint_positions["right_Slider_1"]:.2f}')
                else:
                    self.get_logger().info(f'Unknown arrow key sequence: {key}')
            else:
                self.get_logger().info(f'Unknown key: {key}')
            
            # Publish joint state
            self.publish_joint_state()
    
    def publish_joint_state(self):
        msg = JointState()
        msg.name = []
        msg.position = []
        for name, position in self.joint_positions.items():
            msg.name.append(name)
            msg.position.append(position)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(msg)
    
    def destroy_node(self):
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()