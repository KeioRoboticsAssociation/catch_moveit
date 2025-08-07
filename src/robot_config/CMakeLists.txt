cmake_minimum_required(VERSION 3.22)
project(robot_config)

find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_package()

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
  src/gui.py
  src/publish_collision_mesh.py
  DESTINATION lib/${PROJECT_NAME}
)

# Add the executable for the move_to_pose_cpp node
add_executable(move_to_pose_cpp src/move_to_pose_cpp.cpp)

# Add dependencies for the move_to_pose_cpp node
ament_target_dependencies(
  move_to_pose_cpp
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
)

# Install the move_to_pose_cpp node
install(TARGETS
  move_to_pose_cpp
  DESTINATION lib/${PROJECT_NAME}
)
