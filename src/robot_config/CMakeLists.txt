cmake_minimum_required(VERSION 3.22)
project(robot_config)

find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_servo QUIET)
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
find_package(moveit_planners_chomp QUIET)
find_package(stomp QUIET)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY path DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})


# Install python scripts
install(PROGRAMS
  src/publish_collision_mesh.py
  src/target_pose_router.py
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
  ament_index_cpp
  controller_manager_msgs
)

# Link yaml-cpp library
target_link_libraries(move_to_pose_dual_cpp yaml-cpp)

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

# Export plugin dependencies
# Optional exports if available
if(moveit_planners_chomp_FOUND)
  ament_export_dependencies(moveit_planners_chomp)
endif()
if(stomp_FOUND)
  ament_export_dependencies(stomp)
endif()

ament_package()
