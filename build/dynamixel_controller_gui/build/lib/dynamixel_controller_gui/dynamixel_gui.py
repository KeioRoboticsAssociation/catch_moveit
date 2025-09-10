#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import rclpy
from rclpy.node import Node
from dynamixel_controller.msg import DynamixelController, DynamixelCommand, DynamixelResponse
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import collections

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
        self.root.geometry("1400x700")
        
        # Variables
        self.motor_ids = []
        self.selected_motor_id = tk.IntVar(value=1)
        self.scan_running = False
        
        # Graph tracking variables
        self.tracking_enabled = tk.BooleanVar(value=False)
        self.tracking_motor_id = tk.IntVar(value=1)
        self.auto_read_enabled = tk.BooleanVar(value=False)
        self.max_data_points = 100
        self.time_data = collections.deque(maxlen=self.max_data_points)
        self.target_position_data = collections.deque(maxlen=self.max_data_points)
        self.current_position_data = collections.deque(maxlen=self.max_data_points)
        self.difference_data = collections.deque(maxlen=self.max_data_points)
        self.start_time = time.time()
        self.last_target_position = None
        self.last_current_position = None
        self.last_read_address = None  # Track the last read address for position tracking
        self.auto_read_timer = None
        
        self.setup_gui()
        
    def setup_gui(self):
        # Create main frames
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.columnconfigure(2, weight=1)
        main_frame.rowconfigure(3, weight=1)
        
        # Motor Discovery Frame
        discovery_frame = ttk.LabelFrame(main_frame, text="Motor Discovery", padding="5")
        discovery_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
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
        control_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
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
        
        # PID Gains Control Frame
        gains_frame = ttk.LabelFrame(main_frame, text="PID Gains Control", padding="5")
        gains_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(10, 10))
        gains_frame.columnconfigure(1, weight=1)
        
        # D Gain controls
        ttk.Label(gains_frame, text="D Gain:").grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        self.d_gain_var = tk.IntVar(value=0)
        d_gain_spinbox = ttk.Spinbox(gains_frame, from_=0, to=16383, textvariable=self.d_gain_var, width=10)
        d_gain_spinbox.grid(row=0, column=1, sticky=tk.W, padx=(0, 10))
        
        ttk.Button(gains_frame, text="Read D Gain", command=self.read_d_gain).grid(row=0, column=2, padx=(0, 5))
        ttk.Button(gains_frame, text="Write D Gain", command=self.write_d_gain).grid(row=0, column=3, padx=(0, 5))
        
        # I Gain controls
        ttk.Label(gains_frame, text="I Gain:").grid(row=0, column=4, sticky=tk.W, padx=(10, 5))
        self.i_gain_var = tk.IntVar(value=0)
        i_gain_spinbox = ttk.Spinbox(gains_frame, from_=0, to=16383, textvariable=self.i_gain_var, width=10)
        i_gain_spinbox.grid(row=0, column=5, sticky=tk.W, padx=(0, 10))
        
        ttk.Button(gains_frame, text="Read I Gain", command=self.read_i_gain).grid(row=0, column=6, padx=(0, 5))
        ttk.Button(gains_frame, text="Write I Gain", command=self.write_i_gain).grid(row=0, column=7, padx=(0, 5))
        
        # P Gain controls
        ttk.Label(gains_frame, text="P Gain:").grid(row=1, column=0, sticky=tk.W, padx=(0, 5), pady=(5, 0))
        self.p_gain_var = tk.IntVar(value=0)
        p_gain_spinbox = ttk.Spinbox(gains_frame, from_=0, to=16383, textvariable=self.p_gain_var, width=10)
        p_gain_spinbox.grid(row=1, column=1, sticky=tk.W, padx=(0, 10), pady=(5, 0))
        
        ttk.Button(gains_frame, text="Read P Gain", command=self.read_p_gain).grid(row=1, column=2, padx=(0, 5), pady=(5, 0))
        ttk.Button(gains_frame, text="Write P Gain", command=self.write_p_gain).grid(row=1, column=3, padx=(0, 5), pady=(5, 0))
        
        # Quick preset buttons
        ttk.Label(gains_frame, text="Presets:").grid(row=1, column=4, sticky=tk.W, padx=(10, 5), pady=(5, 0))
        ttk.Button(gains_frame, text="Read All Gains", command=self.read_all_gains).grid(row=1, column=5, padx=(0, 5), pady=(5, 0))
        ttk.Button(gains_frame, text="Soft", command=lambda: self.set_gain_preset("soft")).grid(row=1, column=6, padx=(0, 5), pady=(5, 0))
        ttk.Button(gains_frame, text="Stiff", command=lambda: self.set_gain_preset("stiff")).grid(row=1, column=7, padx=(0, 5), pady=(5, 0))
        
        # Register access frame
        register_frame = ttk.LabelFrame(main_frame, text="Register Access", padding="5")
        register_frame.grid(row=3, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
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
            ("LED", DynamixelController.LED, 1),
            ("D Gain", DynamixelController.POSITION_D_GAIN, 2),
            ("I Gain", DynamixelController.POSITION_I_GAIN, 2),
            ("P Gain", DynamixelController.POSITION_P_GAIN, 2)
        ]
        
        for i, (name, addr, length) in enumerate(common_registers):
            row = i // 2
            col = i % 2
            btn = ttk.Button(common_frame, text=name, 
                           command=lambda a=addr, l=length: self.set_register_and_read(a, l))
            btn.grid(row=row, column=col, padx=2, pady=2, sticky=tk.W)
        
        # Log frame
        log_frame = ttk.LabelFrame(main_frame, text="Communication Log", padding="5")
        log_frame.grid(row=3, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        
        # Log text area
        self.log_text = scrolledtext.ScrolledText(log_frame, width=50, height=20, wrap=tk.WORD)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Clear log button
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).grid(row=1, column=0, pady=(5, 0))
        
        # Position Tracking Graph Frame
        graph_frame = ttk.LabelFrame(main_frame, text="Position Difference Graph", padding="5")
        graph_frame.grid(row=3, column=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        graph_frame.rowconfigure(1, weight=1)
        graph_frame.columnconfigure(0, weight=1)
        
        # Graph controls
        controls_frame = ttk.Frame(graph_frame)
        controls_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        ttk.Checkbutton(controls_frame, text="Enable Tracking", 
                       variable=self.tracking_enabled).grid(row=0, column=0, padx=(0, 5))
        
        ttk.Label(controls_frame, text="Track ID:").grid(row=0, column=1, padx=(5, 2))
        ttk.Spinbox(controls_frame, from_=0, to=13, textvariable=self.tracking_motor_id, 
                   width=5).grid(row=0, column=2, padx=(0, 5))
        
        ttk.Button(controls_frame, text="Clear Graph", 
                  command=self.clear_graph).grid(row=0, column=3, padx=(5, 0))
        
        ttk.Checkbutton(controls_frame, text="Auto-Read Position", 
                       variable=self.auto_read_enabled,
                       command=self.toggle_auto_read).grid(row=0, column=4, padx=(10, 0))
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(6, 4), dpi=80)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel('Time (seconds)')
        self.ax.set_ylabel('Position')
        self.ax.set_title('Position Tracking')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(['Target', 'Current', 'Difference'], loc='upper right')
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Initialize plot lines
        self.target_line, = self.ax.plot([], [], 'r-', label='Target Position', linewidth=2)
        self.current_line, = self.ax.plot([], [], 'b-', label='Current Position', linewidth=2)
        self.diff_line, = self.ax.plot([], [], 'g-', label='Difference (Target - Current)', linewidth=2)
        self.ax.legend()
        
        
    def set_register_and_read(self, address, length):
        """Set register address and length, then read"""
        self.address_var.set(address)
        self.length_var.set(length)
        self.read_register()
        
        # If reading PRESENT_POSITION and tracking is enabled, suggest setting track ID to current motor
        if address == DynamixelController.PRESENT_POSITION and self.tracking_enabled.get():
            current_motor = self.selected_motor_id.get()
            if current_motor != self.tracking_motor_id.get():
                self.tracking_motor_id.set(current_motor)
                self.log_message(f"Graph tracking ID updated to {current_motor}")
        
    def log_message(self, message):
        """Add message to log with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        
    def clear_log(self):
        """Clear the log"""
        self.log_text.delete(1.0, tk.END)
        
    def clear_graph(self):
        """Clear the position tracking graph"""
        self.time_data.clear()
        self.target_position_data.clear()
        self.current_position_data.clear()
        self.difference_data.clear()
        self.start_time = time.time()
        self.last_target_position = None
        self.last_current_position = None
        self.update_graph()
        self.log_message("Graph data cleared")
        
    def update_graph(self):
        """Update the position tracking graph"""
        if not self.tracking_enabled.get():
            return
            
        # Clear the lines
        self.target_line.set_data([], [])
        self.current_line.set_data([], [])
        self.diff_line.set_data([], [])
        
        if len(self.time_data) > 0:
            # Update line data
            self.target_line.set_data(list(self.time_data), list(self.target_position_data))
            self.current_line.set_data(list(self.time_data), list(self.current_position_data))
            self.diff_line.set_data(list(self.time_data), list(self.difference_data))
            
            # Auto-scale the axes
            self.ax.relim()
            self.ax.autoscale_view()
        
        # Redraw the canvas
        self.canvas.draw_idle()
        
    def add_position_data(self, motor_id, target_pos=None, current_pos=None):
        """Add position data to the graph if tracking is enabled for this motor"""
        if not self.tracking_enabled.get() or motor_id != self.tracking_motor_id.get():
            return
            
        current_time = time.time() - self.start_time
        
        # Update stored positions
        if target_pos is not None:
            self.last_target_position = target_pos
        if current_pos is not None:
            self.last_current_position = current_pos
            
        # Only add data point if we have both target and current positions
        if self.last_target_position is not None and self.last_current_position is not None:
            self.time_data.append(current_time)
            self.target_position_data.append(self.last_target_position)
            self.current_position_data.append(self.last_current_position)
            difference = self.last_target_position - self.last_current_position
            self.difference_data.append(difference)
            
            # Update the graph
            self.update_graph()
        
    def toggle_auto_read(self):
        """Toggle automatic position reading"""
        if self.auto_read_enabled.get():
            self.start_auto_read()
            self.log_message("Auto-read position enabled")
        else:
            self.stop_auto_read()
            self.log_message("Auto-read position disabled")
            
    def start_auto_read(self):
        """Start automatic position reading"""
        if self.auto_read_enabled.get() and self.tracking_enabled.get():
            # Read present position of the tracked motor
            motor_id = self.tracking_motor_id.get()
            msg = DynamixelCommand()
            msg.command = DynamixelController.READ_DATA
            msg.ids = [motor_id]
            msg.address = DynamixelController.PRESENT_POSITION
            msg.length = 4
            msg.data = []
            self.tx_publisher.publish(msg)
            
            # Store the read address for position tracking
            self.last_read_address = DynamixelController.PRESENT_POSITION
            
        # Schedule next read
        if self.auto_read_enabled.get():
            self.auto_read_timer = self.root.after(100, self.start_auto_read)  # Read every 100ms
            
    def stop_auto_read(self):
        """Stop automatic position reading"""
        if self.auto_read_timer:
            self.root.after_cancel(self.auto_read_timer)
            self.auto_read_timer = None
    
    def read_d_gain(self):
        """Read D gain from selected motor"""
        motor_id = self.selected_motor_id.get()
        msg = DynamixelCommand()
        msg.command = DynamixelController.READ_DATA
        msg.ids = [motor_id]
        msg.address = DynamixelController.POSITION_D_GAIN
        msg.length = 2  # 2 bytes for gain values
        msg.data = []
        self.tx_publisher.publish(msg)
        self.last_read_address = DynamixelController.POSITION_D_GAIN
        self.log_message(f"READ D Gain sent to motor {motor_id}")
        
    def write_d_gain(self):
        """Write D gain to selected motor"""
        try:
            motor_id = self.selected_motor_id.get()
            gain_value = self.d_gain_var.get()
            
            # Convert value to 2 bytes (little-endian)
            value_bytes = [(gain_value & 0xFF), ((gain_value >> 8) & 0xFF)]
            
            msg = DynamixelCommand()
            msg.command = DynamixelController.WRITE_DATA
            msg.ids = [motor_id]
            msg.address = DynamixelController.POSITION_D_GAIN
            msg.length = 2
            msg.data = value_bytes
            self.tx_publisher.publish(msg)
            self.log_message(f"WRITE D Gain ({gain_value}) sent to motor {motor_id}")
            
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid D gain value: {str(e)}")
            
    def read_i_gain(self):
        """Read I gain from selected motor"""
        motor_id = self.selected_motor_id.get()
        msg = DynamixelCommand()
        msg.command = DynamixelController.READ_DATA
        msg.ids = [motor_id]
        msg.address = DynamixelController.POSITION_I_GAIN
        msg.length = 2
        msg.data = []
        self.tx_publisher.publish(msg)
        self.last_read_address = DynamixelController.POSITION_I_GAIN
        self.log_message(f"READ I Gain sent to motor {motor_id}")
        
    def write_i_gain(self):
        """Write I gain to selected motor"""
        try:
            motor_id = self.selected_motor_id.get()
            gain_value = self.i_gain_var.get()
            
            # Convert value to 2 bytes (little-endian)
            value_bytes = [(gain_value & 0xFF), ((gain_value >> 8) & 0xFF)]
            
            msg = DynamixelCommand()
            msg.command = DynamixelController.WRITE_DATA
            msg.ids = [motor_id]
            msg.address = DynamixelController.POSITION_I_GAIN
            msg.length = 2
            msg.data = value_bytes
            self.tx_publisher.publish(msg)
            self.log_message(f"WRITE I Gain ({gain_value}) sent to motor {motor_id}")
            
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid I gain value: {str(e)}")
            
    def read_p_gain(self):
        """Read P gain from selected motor"""
        motor_id = self.selected_motor_id.get()
        msg = DynamixelCommand()
        msg.command = DynamixelController.READ_DATA
        msg.ids = [motor_id]
        msg.address = DynamixelController.POSITION_P_GAIN
        msg.length = 2
        msg.data = []
        self.tx_publisher.publish(msg)
        self.last_read_address = DynamixelController.POSITION_P_GAIN
        self.log_message(f"READ P Gain sent to motor {motor_id}")
        
    def write_p_gain(self):
        """Write P gain to selected motor"""
        try:
            motor_id = self.selected_motor_id.get()
            gain_value = self.p_gain_var.get()
            
            # Convert value to 2 bytes (little-endian)
            value_bytes = [(gain_value & 0xFF), ((gain_value >> 8) & 0xFF)]
            
            msg = DynamixelCommand()
            msg.command = DynamixelController.WRITE_DATA
            msg.ids = [motor_id]
            msg.address = DynamixelController.POSITION_P_GAIN
            msg.length = 2
            msg.data = value_bytes
            self.tx_publisher.publish(msg)
            self.log_message(f"WRITE P Gain ({gain_value}) sent to motor {motor_id}")
            
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid P gain value: {str(e)}")
            
    def read_all_gains(self):
        """Read all PID gains from selected motor"""
        self.read_d_gain()
        time.sleep(0.02)  # Small delay between commands
        self.read_i_gain()
        time.sleep(0.02)
        self.read_p_gain()
        
    def set_gain_preset(self, preset):
        """Set PID gain presets"""
        if preset == "soft":
            # Soft preset - less aggressive response
            self.d_gain_var.set(100)
            self.i_gain_var.set(0)
            self.p_gain_var.set(800)
            self.log_message("Set soft gain preset (D=100, I=0, P=800)")
        elif preset == "stiff":
            # Stiff preset - more aggressive response  
            self.d_gain_var.set(4000)
            self.i_gain_var.set(0)
            self.p_gain_var.set(800)
            self.log_message("Set stiff gain preset (D=4000, I=0, P=800)")
        
        # Write all gains
        self.write_d_gain()
        time.sleep(0.02)
        self.write_i_gain()
        time.sleep(0.02)
        self.write_p_gain()
        
        
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
        
        # Store the read address for position tracking
        self.last_read_address = address
        
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
            
            # Track target position if writing to GOAL_POSITION
            if address == DynamixelController.GOAL_POSITION and length == 4:
                original_value = int(self.value_var.get())
                self.add_position_data(motor_id, target_pos=original_value)
            
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
                            # Track current position if reading from PRESENT_POSITION
                            if self.last_read_address == DynamixelController.PRESENT_POSITION:
                                self.add_position_data(motor_id, current_pos=signed_value)
                        else:
                            # For positive values, still show both interpretations if it could be a position
                            self.log_message(f"READ response from motor {motor_id}: {value} (signed/unsigned) (0x{value:X})")
                            # Track current position if reading from PRESENT_POSITION
                            if self.last_read_address == DynamixelController.PRESENT_POSITION:
                                self.add_position_data(motor_id, current_pos=value)
                    else:
                        # For 2-byte registers, check if it's a gain register
                        if length == 2:
                            # Update gain variables if reading gain registers
                            if self.last_read_address == DynamixelController.POSITION_D_GAIN:
                                self.d_gain_var.set(value)
                                self.log_message(f"READ D Gain response from motor {motor_id}: {value}")
                            elif self.last_read_address == DynamixelController.POSITION_I_GAIN:
                                self.i_gain_var.set(value)
                                self.log_message(f"READ I Gain response from motor {motor_id}: {value}")
                            elif self.last_read_address == DynamixelController.POSITION_P_GAIN:
                                self.p_gain_var.set(value)
                                self.log_message(f"READ P Gain response from motor {motor_id}: {value}")
                            else:
                                # For other 2-byte registers, show as unsigned
                                self.log_message(f"READ response from motor {motor_id}: {value} (0x{value:X})")
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
        
        # Set up proper shutdown handling
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        
        # Start GUI main loop
        self.root.mainloop()
        
    def shutdown(self):
        """Cleanup and shutdown the application"""
        self.stop_auto_read()
        self.root.quit()
        self.root.destroy()
        
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