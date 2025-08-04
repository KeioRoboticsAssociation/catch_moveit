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
)

# Install the teleop node
install(TARGETS
  joystick_servo
  DESTINATION lib/${PROJECT_NAME}
)

# Install python script
install(PROGRAMS
  src/gui.py
  DESTINATION lib/${PROJECT_NAME}
)
