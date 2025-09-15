#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                            QHBoxLayout, QLabel, QLineEdit, QPushButton,
                            QComboBox, QGroupBox, QGridLayout)
from PyQt5.QtCore import QTimer, Qt
from std_msgs.msg import String
from stm32_mavlink_interface.msg import RobomasterMotorCommand, RobomasterMotorConfig


class SimpleMotorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RoboMaster Motor Control GUI")
        self.setGeometry(100, 100, 600, 500)

        # ROS2 node
        rclpy.init()
        self.node = Node('simple_motor_gui')

        # Publishers
        self.motor_cmd_pub = self.node.create_publisher(
            RobomasterMotorCommand, '/robomaster/motor_command', 10)
        self.pid_cmd_pub = self.node.create_publisher(
            String, '/pid_command', 10)
        self.motor_config_pub = self.node.create_publisher(
            RobomasterMotorConfig, '/robomaster/motor_config', 10)

        # Setup UI
        self.setup_ui()

        # ROS2 timer
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 100Hz

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Motor Control Group
        motor_group = QGroupBox("Motor Control")
        motor_layout = QGridLayout(motor_group)

        # Motor ID selection
        motor_layout.addWidget(QLabel("Motor ID:"), 0, 0)
        self.motor_id_combo = QComboBox()
        self.motor_id_combo.addItems([str(i) for i in range(1, 9)])
        self.motor_id_combo.setCurrentText("5")  # Default to motor 5
        motor_layout.addWidget(self.motor_id_combo, 0, 1)

        # Control mode selection
        motor_layout.addWidget(QLabel("Control Mode:"), 1, 0)
        self.control_mode_combo = QComboBox()
        self.control_mode_combo.addItems(["Position", "Velocity", "Current"])
        self.control_mode_combo.setCurrentText("Position")
        motor_layout.addWidget(self.control_mode_combo, 1, 1)

        # Target value
        motor_layout.addWidget(QLabel("Target Value:"), 2, 0)
        self.target_value_edit = QLineEdit("0.0")
        motor_layout.addWidget(self.target_value_edit, 2, 1)

        # Unit label
        self.unit_label = QLabel("rad")
        motor_layout.addWidget(self.unit_label, 2, 2)

        # Enable checkbox
        self.enable_button = QPushButton("Disable Motor")
        self.enable_button.setCheckable(True)
        self.enable_button.setChecked(True)  # Default to enabled
        self.enable_button.setStyleSheet("background-color: green; color: white;")
        self.enable_button.clicked.connect(self.toggle_motor)
        motor_layout.addWidget(self.enable_button, 3, 0, 1, 2)

        # Send command button
        send_button = QPushButton("Send Command")
        send_button.clicked.connect(self.send_motor_command)
        motor_layout.addWidget(send_button, 3, 2)

        # Emergency stop
        emergency_button = QPushButton("EMERGENCY STOP")
        emergency_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        emergency_button.clicked.connect(self.emergency_stop)
        motor_layout.addWidget(emergency_button, 4, 0, 1, 3)

        main_layout.addWidget(motor_group)

        # PID Configuration Group
        pid_group = QGroupBox("PID Configuration")
        pid_layout = QGridLayout(pid_group)

        # Position PID
        pid_layout.addWidget(QLabel("Position PID"), 0, 0, 1, 3)
        pid_layout.addWidget(QLabel("Kp:"), 1, 0)
        self.pos_kp_edit = QLineEdit("10.0")
        pid_layout.addWidget(self.pos_kp_edit, 1, 1)
        pos_kp_button = QPushButton("Set")
        pos_kp_button.clicked.connect(lambda: self.send_pid_command("POS_KP", self.pos_kp_edit.text()))
        pid_layout.addWidget(pos_kp_button, 1, 2)

        pid_layout.addWidget(QLabel("Ki:"), 2, 0)
        self.pos_ki_edit = QLineEdit("0.1")
        pid_layout.addWidget(self.pos_ki_edit, 2, 1)
        pos_ki_button = QPushButton("Set")
        pos_ki_button.clicked.connect(lambda: self.send_pid_command("POS_KI", self.pos_ki_edit.text()))
        pid_layout.addWidget(pos_ki_button, 2, 2)

        pid_layout.addWidget(QLabel("Kd:"), 3, 0)
        self.pos_kd_edit = QLineEdit("0.5")
        pid_layout.addWidget(self.pos_kd_edit, 3, 1)
        pos_kd_button = QPushButton("Set")
        pos_kd_button.clicked.connect(lambda: self.send_pid_command("POS_KD", self.pos_kd_edit.text()))
        pid_layout.addWidget(pos_kd_button, 3, 2)

        # Velocity PID
        pid_layout.addWidget(QLabel("Velocity PID"), 4, 0, 1, 3)
        pid_layout.addWidget(QLabel("Kp:"), 5, 0)
        self.vel_kp_edit = QLineEdit("50.0")
        pid_layout.addWidget(self.vel_kp_edit, 5, 1)
        vel_kp_button = QPushButton("Set")
        vel_kp_button.clicked.connect(lambda: self.send_pid_command("VEL_KP", self.vel_kp_edit.text()))
        pid_layout.addWidget(vel_kp_button, 5, 2)

        pid_layout.addWidget(QLabel("Ki:"), 6, 0)
        self.vel_ki_edit = QLineEdit("0.5")
        pid_layout.addWidget(self.vel_ki_edit, 6, 1)
        vel_ki_button = QPushButton("Set")
        vel_ki_button.clicked.connect(lambda: self.send_pid_command("VEL_KI", self.vel_ki_edit.text()))
        pid_layout.addWidget(vel_ki_button, 6, 2)

        pid_layout.addWidget(QLabel("Kd:"), 7, 0)
        self.vel_kd_edit = QLineEdit("0.1")
        pid_layout.addWidget(self.vel_kd_edit, 7, 1)
        vel_kd_button = QPushButton("Set")
        vel_kd_button.clicked.connect(lambda: self.send_pid_command("VEL_KD", self.vel_kd_edit.text()))
        pid_layout.addWidget(vel_kd_button, 7, 2)

        # Send all PID button
        send_all_pid_button = QPushButton("Send All PID Gains")
        send_all_pid_button.setStyleSheet("background-color: blue; color: white; font-weight: bold;")
        send_all_pid_button.clicked.connect(self.send_all_pid_gains)
        pid_layout.addWidget(send_all_pid_button, 8, 0, 1, 3)

        main_layout.addWidget(pid_group)

        # Connect control mode change to update unit label
        self.control_mode_combo.currentTextChanged.connect(self.update_unit_label)

    def update_unit_label(self):
        mode = self.control_mode_combo.currentText()
        if mode == "Position":
            self.unit_label.setText("rad")
        elif mode == "Velocity":
            self.unit_label.setText("rps")
        elif mode == "Current":
            self.unit_label.setText("mA")

    def toggle_motor(self):
        if self.enable_button.isChecked():
            self.enable_button.setText("Disable Motor")
            self.enable_button.setStyleSheet("background-color: green; color: white;")
        else:
            self.enable_button.setText("Enable Motor")
            self.enable_button.setStyleSheet("")

    def send_motor_command(self):
        try:
            motor_id = int(self.motor_id_combo.currentText())
            target_value = float(self.target_value_edit.text())
            mode = self.control_mode_combo.currentText()
            enabled = self.enable_button.isChecked()

            cmd_msg = RobomasterMotorCommand()
            cmd_msg.header.stamp = self.node.get_clock().now().to_msg()
            cmd_msg.motor_id = motor_id
            cmd_msg.enabled = enabled

            if mode == "Position":
                cmd_msg.control_mode = 2  # CONTROL_MODE_POSITION
                cmd_msg.target_position_rad = target_value
            elif mode == "Velocity":
                cmd_msg.control_mode = 1  # CONTROL_MODE_VELOCITY
                cmd_msg.target_velocity_rps = target_value
            elif mode == "Current":
                cmd_msg.control_mode = 0  # CONTROL_MODE_CURRENT
                cmd_msg.target_current_ma = int(target_value)

            self.motor_cmd_pub.publish(cmd_msg)
            print(f"Sent command: Motor {motor_id}, Mode: {mode}, Value: {target_value}, Enabled: {enabled}")

        except ValueError as e:
            print(f"Invalid input: {e}")

    def send_pid_command(self, param_name, value):
        try:
            motor_id = int(self.motor_id_combo.currentText())
            pid_value = float(value)

            pid_msg = String()
            pid_msg.data = f"{motor_id},{param_name},{pid_value}"

            self.pid_cmd_pub.publish(pid_msg)
            print(f"Sent PID command: Motor {motor_id}, {param_name} = {pid_value}")

        except ValueError as e:
            print(f"Invalid PID value: {e}")

    def send_all_pid_gains(self):
        """Send all PID gains at once using single RobomasterMotorConfig message"""
        try:
            motor_id = int(self.motor_id_combo.currentText())

            # Create motor config message
            config_msg = RobomasterMotorConfig()
            config_msg.header.stamp = self.node.get_clock().now().to_msg()
            config_msg.motor_id = motor_id

            # Set all PID values
            try:
                config_msg.position_kp = float(self.pos_kp_edit.text())
                config_msg.position_ki = float(self.pos_ki_edit.text())
                config_msg.position_kd = float(self.pos_kd_edit.text())
                config_msg.velocity_kp = float(self.vel_kp_edit.text())
                config_msg.velocity_ki = float(self.vel_ki_edit.text())
                config_msg.velocity_kd = float(self.vel_kd_edit.text())

                # Send single message with all PID gains
                self.motor_config_pub.publish(config_msg)

                print(f"Sent complete PID config for Motor {motor_id}:")
                print(f"  Position PID: Kp={config_msg.position_kp}, Ki={config_msg.position_ki}, Kd={config_msg.position_kd}")
                print(f"  Velocity PID: Kp={config_msg.velocity_kp}, Ki={config_msg.velocity_ki}, Kd={config_msg.velocity_kd}")

            except ValueError as e:
                print(f"Invalid PID values: {e}")

        except ValueError as e:
            print(f"Invalid motor ID: {e}")

    def emergency_stop(self):
        # Send stop command to all motors
        for motor_id in range(1, 9):
            cmd_msg = RobomasterMotorCommand()
            cmd_msg.header.stamp = self.node.get_clock().now().to_msg()
            cmd_msg.motor_id = motor_id
            cmd_msg.enabled = False
            cmd_msg.emergency_stop = True
            cmd_msg.control_mode = 0
            cmd_msg.target_current_ma = 0

            self.motor_cmd_pub.publish(cmd_msg)

        print("EMERGENCY STOP - All motors disabled")

        # Reset enable button
        self.enable_button.setChecked(False)
        self.toggle_motor()

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.001)

    def closeEvent(self, event):
        self.emergency_stop()  # Safety: stop all motors on close
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)

    try:
        gui = SimpleMotorGUI()
        gui.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    main()