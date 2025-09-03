#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import rclpy
from rclpy.node import Node
from dynamixel_controller.msg import DynamixelController, DynamixelCommand, DynamixelResponse

class DynamixelGUI(Node):
    def __init__(self):
        super().__init__('dynamixel_gui')
        
        # ROS2 publishers and subscribers
        self.tx_publisher = self.create_publisher(DynamixelCommand, 'dynamixel_tx', 10)
        self.rx_subscription = self.create_subscription(
            DynamixelResponse,
            'dynamixel_rx',
            self.rx_callback,
            10
        )
        
        # GUI setup
        self.root = tk.Tk()
        self.root.title("Dynamixel Controller GUI - Similar to Dynamixel Wizard")
        self.root.geometry("1000x700")
        
        # Variables
        self.motor_ids = []
        self.selected_motor_id = tk.IntVar(value=1)
        self.scan_running = False
        
        self.setup_gui()
        
    def setup_gui(self):
        # Create main frames
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)
        
        # Motor Discovery Frame
        discovery_frame = ttk.LabelFrame(main_frame, text="Motor Discovery", padding="5")
        discovery_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Scan buttons
        ttk.Button(discovery_frame, text="Scan Motors", command=self.scan_motors).grid(row=0, column=0, padx=(0, 5))
        ttk.Button(discovery_frame, text="Ping All", command=self.ping_all_motors).grid(row=0, column=1, padx=(0, 5))
        ttk.Button(discovery_frame, text="REBOOTALL", command=self.reboot_all_motors, 
                  style="Accent.TButton").grid(row=0, column=2, padx=(0, 5))
        ttk.Button(discovery_frame, text="TORQUE ALL ON", command=self.torque_all_on).grid(row=0, column=5, padx=(0, 5))
        ttk.Button(discovery_frame, text="TORQUE ALL OFF", command=self.torque_all_off).grid(row=0, column=6, padx=(0, 5))
        
        # Motors found display
        ttk.Label(discovery_frame, text="Found Motors:").grid(row=0, column=7, padx=(20, 5))
        self.motors_found_var = tk.StringVar(value="None")
        ttk.Label(discovery_frame, textvariable=self.motors_found_var).grid(row=0, column=8)
        
        # Motor Control Frame
        control_frame = ttk.LabelFrame(main_frame, text="Motor Control", padding="5")
        control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        control_frame.columnconfigure(1, weight=1)
        
        # Motor ID selection
        ttk.Label(control_frame, text="Motor ID:").grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        motor_id_spinbox = ttk.Spinbox(control_frame, from_=0, to=13, textvariable=self.selected_motor_id, width=10)
        motor_id_spinbox.grid(row=0, column=1, sticky=tk.W, padx=(0, 20))
        
        # Quick actions
        ttk.Button(control_frame, text="PING", command=self.ping_motor).grid(row=0, column=2, padx=(0, 5))
        ttk.Button(control_frame, text="LED ON", command=self.led_on).grid(row=0, column=3, padx=(0, 5))
        ttk.Button(control_frame, text="LED OFF", command=self.led_off).grid(row=0, column=4, padx=(0, 5))
        ttk.Button(control_frame, text="Torque ON", command=self.torque_on).grid(row=0, column=5, padx=(0, 5))
        ttk.Button(control_frame, text="Torque OFF", command=self.torque_off).grid(row=0, column=6, padx=(0, 5))
        ttk.Button(control_frame, text="REBOOT", command=self.reboot_motor, 
                  style="Accent.TButton").grid(row=0, column=7, padx=(0, 5))
        
        # Register access frame
        register_frame = ttk.LabelFrame(main_frame, text="Register Access", padding="5")
        register_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        register_frame.rowconfigure(4, weight=1)
        
        # Register address
        ttk.Label(register_frame, text="Address:").grid(row=0, column=0, sticky=tk.W)
        self.address_var = tk.IntVar(value=132)  # Default to PRESENT_POSITION
        ttk.Spinbox(register_frame, from_=0, to=255, textvariable=self.address_var, width=10).grid(row=0, column=1, sticky=tk.W, pady=(0, 5))
        
        # Data length
        ttk.Label(register_frame, text="Length:").grid(row=1, column=0, sticky=tk.W)
        self.length_var = tk.IntVar(value=4)
        ttk.Spinbox(register_frame, from_=1, to=8, textvariable=self.length_var, width=10).grid(row=1, column=1, sticky=tk.W, pady=(0, 5))
        
        # Read/Write buttons
        ttk.Button(register_frame, text="Read", command=self.read_register).grid(row=2, column=0, pady=(5, 0), sticky=tk.W)
        ttk.Button(register_frame, text="Write", command=self.write_register).grid(row=2, column=1, pady=(5, 0), sticky=tk.W)
        
        # Value input for writing
        ttk.Label(register_frame, text="Value:").grid(row=3, column=0, sticky=tk.W)
        self.value_var = tk.StringVar(value="0")
        ttk.Entry(register_frame, textvariable=self.value_var, width=15).grid(row=3, column=1, sticky=tk.W)
        
        # Common registers frame
        common_frame = ttk.LabelFrame(register_frame, text="Common Registers", padding="5")
        common_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(10, 0))
        
        # Create buttons for common registers
        common_registers = [
            ("Present Position", DynamixelController.PRESENT_POSITION, 4),
            ("Present Velocity", DynamixelController.PRESENT_VELOCITY, 4),
            ("Present Current", DynamixelController.PRESENT_CURRENT, 2),
            ("Present Temperature", DynamixelController.PRESENT_TEMPERATURE, 1),
            ("Goal Position", DynamixelController.GOAL_POSITION, 4),
            ("Goal Velocity", DynamixelController.GOAL_VELOCITY, 4),
            ("Torque Enable", DynamixelController.TORQUE_ENABLE, 1),
            ("LED", DynamixelController.LED, 1)
        ]
        
        for i, (name, addr, length) in enumerate(common_registers):
            row = i // 2
            col = i % 2
            btn = ttk.Button(common_frame, text=name, 
                           command=lambda a=addr, l=length: self.set_register_and_read(a, l))
            btn.grid(row=row, column=col, padx=2, pady=2, sticky=tk.W)
        
        # Log frame
        log_frame = ttk.LabelFrame(main_frame, text="Communication Log", padding="5")
        log_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        
        # Log text area
        self.log_text = scrolledtext.ScrolledText(log_frame, width=50, height=20, wrap=tk.WORD)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Clear log button
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).grid(row=1, column=0, pady=(5, 0))
        
    def set_register_and_read(self, address, length):
        """Set register address and length, then read"""
        self.address_var.set(address)
        self.length_var.set(length)
        self.read_register()
        
    def log_message(self, message):
        """Add message to log with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        
    def clear_log(self):
        """Clear the log"""
        self.log_text.delete(1.0, tk.END)
        
    def scan_motors(self):
        """Scan for motors by pinging all IDs"""
        if self.scan_running:
            return
            
        self.scan_running = True
        self.motor_ids = []
        self.log_message("Starting motor scan...")
        
        # Start scan in separate thread to avoid blocking GUI
        threading.Thread(target=self._scan_thread, daemon=True).start()
        
    def _scan_thread(self):
        """Background thread for scanning motors"""
        found_motors = []
        
        for motor_id in range(0, 14):  # Scan IDs 0-13
            msg = DynamixelCommand()
            msg.command = DynamixelController.PING
            msg.ids = [motor_id]
            msg.address = 0
            msg.length = 0
            msg.data = []
            self.tx_publisher.publish(msg)
            
            # Small delay between pings
            time.sleep(0.01)
            
        # Update GUI in main thread
        self.root.after(2000, lambda: self._finish_scan(found_motors))
        
    def _finish_scan(self, found_motors):
        """Finish the scan and update GUI"""
        self.scan_running = False
        if self.motor_ids:
            motors_str = ", ".join(map(str, sorted(self.motor_ids)))
            self.motors_found_var.set(motors_str)
            self.log_message(f"Scan complete. Found motors: {motors_str}")
        else:
            self.motors_found_var.set("None")
            self.log_message("Scan complete. No motors found.")
            
    def ping_all_motors(self):
        """Ping all discovered motors"""
        if not self.motor_ids:
            messagebox.showwarning("Warning", "No motors found. Please scan first.")
            return
            
        for motor_id in self.motor_ids:
            msg = DynamixelCommand()
            msg.command = DynamixelController.PING
            msg.ids = [motor_id]
            msg.address = 0
            msg.length = 0
            msg.data = []
            self.tx_publisher.publish(msg)
            self.log_message(f"PING sent to motor {motor_id}")
            
    def ping_motor(self):
        """Ping selected motor"""
        motor_id = self.selected_motor_id.get()
        msg = DynamixelCommand()
        msg.command = DynamixelController.PING
        msg.ids = [motor_id]
        msg.address = 0
        msg.length = 0
        msg.data = []
        self.tx_publisher.publish(msg)
        self.log_message(f"PING sent to motor {motor_id}")
        
    def led_on(self):
        """Turn LED on"""
        motor_id = self.selected_motor_id.get()
        msg = DynamixelCommand()
        msg.command = DynamixelController.WRITE_DATA
        msg.ids = [motor_id]
        msg.address = DynamixelController.LED
        msg.length = 1
        msg.data = [1]
        self.tx_publisher.publish(msg)
        self.log_message(f"LED ON sent to motor {motor_id}")
        
    def led_off(self):
        """Turn LED off"""
        motor_id = self.selected_motor_id.get()
        msg = DynamixelCommand()
        msg.command = DynamixelController.WRITE_DATA
        msg.ids = [motor_id]
        msg.address = DynamixelController.LED
        msg.length = 1
        msg.data = [0]
        self.tx_publisher.publish(msg)
        self.log_message(f"LED OFF sent to motor {motor_id}")
        
    def torque_on(self):
        """Enable torque"""
        motor_id = self.selected_motor_id.get()
        msg = DynamixelCommand()
        msg.command = DynamixelController.WRITE_DATA
        msg.ids = [motor_id]
        msg.address = DynamixelController.TORQUE_ENABLE
        msg.length = 1
        msg.data = [1]
        self.tx_publisher.publish(msg)
        self.log_message(f"Torque ON sent to motor {motor_id}")
        
    def torque_off(self):
        """Disable torque"""
        motor_id = self.selected_motor_id.get()
        msg = DynamixelCommand()
        msg.command = DynamixelController.WRITE_DATA
        msg.ids = [motor_id]
        msg.address = DynamixelController.TORQUE_ENABLE
        msg.length = 1
        msg.data = [0]
        self.tx_publisher.publish(msg)
        self.log_message(f"Torque OFF sent to motor {motor_id}")
        
    def reboot_motor(self):
        """Reboot selected motor"""
        motor_id = self.selected_motor_id.get()
        
        # Confirm reboot action with user
        result = messagebox.askyesno(
            "Confirm Reboot", 
            f"Are you sure you want to reboot motor ID {motor_id}?\n\n"
            "The motor will restart and may lose its current position.\n"
            "This action cannot be undone."
        )
        
        if result:
            msg = DynamixelCommand()
            msg.command = DynamixelController.REBOOT
            msg.ids = [motor_id]
            msg.address = 0  # Not used for REBOOT command
            msg.length = 0   # Not used for REBOOT command
            msg.data = []    # Not used for REBOOT command
            self.tx_publisher.publish(msg)
            self.log_message(f"REBOOT command sent to motor {motor_id}")
        else:
            self.log_message(f"REBOOT cancelled for motor {motor_id}")
    
    def reboot_all_motors(self):
        """Reboot all discovered motors"""
        if not self.motor_ids:
            messagebox.showwarning("Warning", "No motors found. Please scan first.")
            return
            
        motor_list = ", ".join(map(str, sorted(self.motor_ids)))
        self.log_message(f"REBOOT ALL started for motors: {motor_list}")
        
        for motor_id in self.motor_ids:
            msg = DynamixelCommand()
            msg.command = DynamixelController.REBOOT
            msg.ids = [motor_id]
            msg.address = 0  # Not used for REBOOT command
            msg.length = 0   # Not used for REBOOT command
            msg.data = []    # Not used for REBOOT command
            self.tx_publisher.publish(msg)
            self.log_message(f"REBOOT command sent to motor {motor_id}")
            
            # Small delay between reboots to avoid overwhelming the bus
            time.sleep(0.05)
    
    def torque_all_on(self):
        """Enable torque for all discovered motors"""
        if not self.motor_ids:
            messagebox.showwarning("Warning", "No motors found. Please scan first.")
            return
            
        motor_list = ", ".join(map(str, sorted(self.motor_ids)))
        self.log_message(f"TORQUE ALL ON started for motors: {motor_list}")
        
        for motor_id in self.motor_ids:
            msg = DynamixelCommand()
            msg.command = DynamixelController.WRITE_DATA
            msg.ids = [motor_id]
            msg.address = DynamixelController.TORQUE_ENABLE
            msg.length = 1
            msg.data = [1]
            self.tx_publisher.publish(msg)
            self.log_message(f"Torque ON sent to motor {motor_id}")
            
            # Small delay between commands to avoid overwhelming the bus
            time.sleep(0.02)
    
    def torque_all_off(self):
        """Disable torque for all discovered motors"""
        if not self.motor_ids:
            messagebox.showwarning("Warning", "No motors found. Please scan first.")
            return
            
        motor_list = ", ".join(map(str, sorted(self.motor_ids)))
        self.log_message(f"TORQUE ALL OFF started for motors: {motor_list}")
        
        for motor_id in self.motor_ids:
            msg = DynamixelCommand()
            msg.command = DynamixelController.WRITE_DATA
            msg.ids = [motor_id]
            msg.address = DynamixelController.TORQUE_ENABLE
            msg.length = 1
            msg.data = [0]
            self.tx_publisher.publish(msg)
            self.log_message(f"Torque OFF sent to motor {motor_id}")
            
            # Small delay between commands to avoid overwhelming the bus
            time.sleep(0.02)
        
    def read_register(self):
        """Read register value"""
        motor_id = self.selected_motor_id.get()
        address = self.address_var.get()
        length = self.length_var.get()
        
        msg = DynamixelCommand()
        msg.command = DynamixelController.READ_DATA
        msg.ids = [motor_id]
        msg.address = address
        msg.length = length
        msg.data = []
        self.tx_publisher.publish(msg)
        self.log_message(f"READ sent to motor {motor_id}, address {address}, length {length}")
        
    def write_register(self):
        """Write register value"""
        try:
            motor_id = self.selected_motor_id.get()
            address = self.address_var.get()
            length = self.length_var.get()
            value = int(self.value_var.get())
            
            # Convert value to bytes with signed support for position registers
            if length == 4 and address in [DynamixelController.PRESENT_POSITION, DynamixelController.GOAL_POSITION]:
                # For position registers, support signed 32-bit integers (Extended Position Control)
                if value < 0:
                    # Convert negative value to unsigned representation
                    value = value + 4294967296  # 2^32
                elif value > 4294967295:  # 2^32 - 1
                    raise ValueError("Value too large for 32-bit register")
                
                # Convert to bytes (little-endian)
                value_bytes = value.to_bytes(4, 'little', signed=False)
                value_bytes = list(value_bytes)
                
                # Log both signed and unsigned interpretation
                original_value = int(self.value_var.get())
                if original_value < 0:
                    self.log_message(f"WRITE sent to motor {motor_id}, address {address}, value {original_value} (signed) / {value} (unsigned)")
                else:
                    self.log_message(f"WRITE sent to motor {motor_id}, address {address}, value {value}")
            else:
                # Standard unsigned conversion for other registers
                if value < 0:
                    raise ValueError("Negative values not supported for this register")
                
                # Convert value to bytes (little-endian)
                value_bytes = []
                for i in range(length):
                    value_bytes.append((value >> (8 * i)) & 0xFF)
                    
                self.log_message(f"WRITE sent to motor {motor_id}, address {address}, value {value}")
            
            msg = DynamixelCommand()
            msg.command = DynamixelController.WRITE_DATA
            msg.ids = [motor_id]
            msg.address = address
            msg.length = length
            msg.data = value_bytes
            self.tx_publisher.publish(msg)
            
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid value: {str(e)}")
            
    def rx_callback(self, msg):
        """Handle received messages from dynamixel_controller"""
        instruction = msg.command
        
        for i, motor_id in enumerate(msg.ids):
            error = msg.error[i] if i < len(msg.error) else 255
            
            if instruction == DynamixelController.PING:
                if error == 0:  # Success
                    if motor_id not in self.motor_ids:
                        self.motor_ids.append(motor_id)
                    self.log_message(f"PING response from motor {motor_id}: Success")
                else:
                    self.log_message(f"PING response from motor {motor_id}: Error {error}")
                    
            elif instruction == DynamixelController.READ_DATA:
                if error == 0 and msg.data:  # Success with data
                    # Convert bytes to integer (little-endian)
                    value = 0
                    data_start_index = i * len(msg.data) // len(msg.ids) if len(msg.ids) > 1 else 0
                    data_end_index = data_start_index + len(msg.data) // len(msg.ids) if len(msg.ids) > 1 else len(msg.data)
                    
                    # Get the data for this specific motor
                    motor_data = msg.data[data_start_index:data_end_index] if len(msg.ids) > 1 else msg.data
                    
                    for j, byte in enumerate(motor_data):
                        value |= byte << (8 * j)
                    
                    # Check if this is a position register that might be signed (Extended Position Control)
                    length = len(motor_data)
                    
                    # For 4-byte registers, always show both signed and unsigned interpretation for position-related values
                    if length == 4:
                        # Convert to signed 32-bit if the value is in the upper half of uint32 range
                        if value > 2147483647:  # 2^31 - 1
                            signed_value = value - 4294967296  # 2^32
                            self.log_message(f"READ response from motor {motor_id}: {signed_value} (signed) / {value} (unsigned) (0x{value:X})")
                        else:
                            # For positive values, still show both interpretations if it could be a position
                            self.log_message(f"READ response from motor {motor_id}: {value} (signed/unsigned) (0x{value:X})")
                    else:
                        # For other data lengths, show as unsigned
                        self.log_message(f"READ response from motor {motor_id}: {value} (0x{value:X})")
                else:
                    self.log_message(f"READ response from motor {motor_id}: Error {error}")
                    
            elif instruction == DynamixelController.WRITE_DATA:
                if error == 0:
                    self.log_message(f"WRITE response from motor {motor_id}: Success")
                else:
                    self.log_message(f"WRITE response from motor {motor_id}: Error {error}")
                    
            elif instruction == DynamixelController.REBOOT:
                if error == 0:
                    self.log_message(f"REBOOT response from motor {motor_id}: Success - Motor rebooted")
                else:
                    self.log_message(f"REBOOT response from motor {motor_id}: Error {error}")
                    
            else:
                self.log_message(f"Unknown response from motor {motor_id}: command={instruction}, error={error}")
            
    def run(self):
        """Run the GUI application"""
        # Start ROS2 spinning in separate thread
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()
        
        # Start GUI main loop
        self.root.mainloop()
        
    def _ros_spin(self):
        """ROS2 spinning thread"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        gui = DynamixelGUI()
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()