#!/bin/bash

# Test script for dual arm realtime control

echo "Testing Dual Arm Realtime Control"

# Start the simple dual arm control node
echo "Starting simple dual arm control node..."
ros2 run robot_config simple_dual_arm_control &
CONTROL_PID=$!

# Wait for node to initialize
sleep 5

echo "Testing left arm forward movement..."
ros2 topic pub --once /left_arm_realtime_control geometry_msgs/msg/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

sleep 2

echo "Testing right arm rotation..."
ros2 topic pub --once /right_arm_realtime_control geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'

sleep 2

echo "Stopping movement..."
ros2 topic pub --once /left_arm_realtime_control geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

ros2 topic pub --once /right_arm_realtime_control geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

echo "Test completed. Stopping node..."
kill $CONTROL_PID

echo "Done!"
