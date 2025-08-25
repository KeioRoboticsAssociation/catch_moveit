#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dynamixel_controller::dynamixel_controller__rosidl_generator_py" for configuration ""
set_property(TARGET dynamixel_controller::dynamixel_controller__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(dynamixel_controller::dynamixel_controller__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdynamixel_controller__rosidl_generator_py.so"
  IMPORTED_SONAME_NOCONFIG "libdynamixel_controller__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS dynamixel_controller::dynamixel_controller__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_dynamixel_controller::dynamixel_controller__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libdynamixel_controller__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
