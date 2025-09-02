cmake_minimum_required(VERSION 3.22)
project(robot_config)

find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Add the executable for the teleop node
add_executable(joystick_servo src/joystick_servo.cpp)

# Add dependencies for the teleop node
ament_target_dependencies(
  joystick_servo
  rclcpp
  sensor_msgs
  geometry_msgs
  control_msgs
  std_srvs
  moveit_msgs
  moveit_servo
  tf2_ros
)

# Install the teleop node
install(TARGETS
  joystick_servo
  DESTINATION lib/${PROJECT_NAME}
)

# Install python script
install(PROGRAMS
  src/publish_collision_mesh.py
  DESTINATION lib/${PROJECT_NAME}
)


# Add the executable for the move_to_pose_dual_cpp node
add_executable(move_to_pose_dual_cpp src/move_to_pose_dual_cpp.cpp)

# Add dependencies for the move_to_pose_dual_cpp node
ament_target_dependencies(
  move_to_pose_dual_cpp
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  control_msgs
)

# Install the move_to_pose_dual_cpp node
install(TARGETS
  move_to_pose_dual_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Add the executable for the pose_command_publisher node
add_executable(pose_command_publisher src/pose_command_publisher.cpp)

# Add dependencies for the pose_command_publisher node
ament_target_dependencies(
  pose_command_publisher
  rclcpp
  std_msgs
)

# Link yaml-cpp library
target_link_libraries(pose_command_publisher yaml-cpp)

# Install the pose_command_publisher node
install(TARGETS
  pose_command_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Add the executable for the dual_arm_realtime_control node
add_executable(dual_arm_realtime_control src/dual_arm_realtime_control.cpp)

# Add dependencies for the dual_arm_realtime_control node
ament_target_dependencies(
  dual_arm_realtime_control
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  geometry_msgs
  tf2_geometry_msgs
)

# Install the dual_arm_realtime_control node
install(TARGETS
  dual_arm_realtime_control
  DESTINATION lib/${PROJECT_NAME}
)

# Add the executable for the dual_arm_cartesian_control node
add_executable(dual_arm_cartesian_control src/dual_arm_cartesian_control.cpp)

# Add dependencies for the dual_arm_cartesian_control node
ament_target_dependencies(
  dual_arm_cartesian_control
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  geometry_msgs
  tf2_geometry_msgs
  moveit_msgs
)

# Install the dual_arm_cartesian_control node
install(TARGETS
  dual_arm_cartesian_control
  DESTINATION lib/${PROJECT_NAME}
)

# Add the executable for the simple_dual_arm_control node
add_executable(simple_dual_arm_control src/simple_dual_arm_control.cpp)

# Add dependencies for the simple_dual_arm_control node
ament_target_dependencies(
  simple_dual_arm_control
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  geometry_msgs
  tf2_geometry_msgs
)

# Install the simple_dual_arm_control node
install(TARGETS
  simple_dual_arm_control
  DESTINATION lib/${PROJECT_NAME}
)

# Add the executable for the dual_arm_servo_controller node
add_executable(dual_arm_servo_controller src/dual_arm_servo_controller.cpp)

# Add dependencies for the dual_arm_servo_controller node
ament_target_dependencies(
  dual_arm_servo_controller
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)

# Install the dual_arm_servo_controller node
install(TARGETS
  dual_arm_servo_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
