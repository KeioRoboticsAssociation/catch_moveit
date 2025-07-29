#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "moveit_servo::moveit_servo_lib" for configuration "Debug"
set_property(TARGET moveit_servo::moveit_servo_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(moveit_servo::moveit_servo_lib PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmoveit_servo_lib.so.2.5.9"
  IMPORTED_SONAME_DEBUG "libmoveit_servo_lib.so.2.5.9"
  )

list(APPEND _cmake_import_check_targets moveit_servo::moveit_servo_lib )
list(APPEND _cmake_import_check_files_for_moveit_servo::moveit_servo_lib "${_IMPORT_PREFIX}/lib/libmoveit_servo_lib.so.2.5.9" )

# Import target "moveit_servo::moveit_servo_lib_parameters" for configuration "Debug"
set_property(TARGET moveit_servo::moveit_servo_lib_parameters APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(moveit_servo::moveit_servo_lib_parameters PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmoveit_servo_lib_parameters.so.2.5.9"
  IMPORTED_SONAME_DEBUG "libmoveit_servo_lib_parameters.so.2.5.9"
  )

list(APPEND _cmake_import_check_targets moveit_servo::moveit_servo_lib_parameters )
list(APPEND _cmake_import_check_files_for_moveit_servo::moveit_servo_lib_parameters "${_IMPORT_PREFIX}/lib/libmoveit_servo_lib_parameters.so.2.5.9" )

# Import target "moveit_servo::pose_tracking" for configuration "Debug"
set_property(TARGET moveit_servo::pose_tracking APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(moveit_servo::pose_tracking PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libpose_tracking.so"
  IMPORTED_SONAME_DEBUG "libpose_tracking.so"
  )

list(APPEND _cmake_import_check_targets moveit_servo::pose_tracking )
list(APPEND _cmake_import_check_files_for_moveit_servo::pose_tracking "${_IMPORT_PREFIX}/lib/libpose_tracking.so" )

# Import target "moveit_servo::servo_node" for configuration "Debug"
set_property(TARGET moveit_servo::servo_node APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(moveit_servo::servo_node PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libservo_node.so"
  IMPORTED_SONAME_DEBUG "libservo_node.so"
  )

list(APPEND _cmake_import_check_targets moveit_servo::servo_node )
list(APPEND _cmake_import_check_files_for_moveit_servo::servo_node "${_IMPORT_PREFIX}/lib/libservo_node.so" )

# Import target "moveit_servo::servo_controller_input" for configuration "Debug"
set_property(TARGET moveit_servo::servo_controller_input APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(moveit_servo::servo_controller_input PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libservo_controller_input.so"
  IMPORTED_SONAME_DEBUG "libservo_controller_input.so"
  )

list(APPEND _cmake_import_check_targets moveit_servo::servo_controller_input )
list(APPEND _cmake_import_check_files_for_moveit_servo::servo_controller_input "${_IMPORT_PREFIX}/lib/libservo_controller_input.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
