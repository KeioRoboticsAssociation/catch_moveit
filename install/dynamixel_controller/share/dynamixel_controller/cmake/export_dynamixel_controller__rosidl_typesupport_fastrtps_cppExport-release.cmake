#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dynamixel_controller::dynamixel_controller__rosidl_typesupport_fastrtps_cpp" for configuration "Release"
set_property(TARGET dynamixel_controller::dynamixel_controller__rosidl_typesupport_fastrtps_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(dynamixel_controller::dynamixel_controller__rosidl_typesupport_fastrtps_cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdynamixel_controller__rosidl_typesupport_fastrtps_cpp.so"
  IMPORTED_SONAME_RELEASE "libdynamixel_controller__rosidl_typesupport_fastrtps_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS dynamixel_controller::dynamixel_controller__rosidl_typesupport_fastrtps_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_dynamixel_controller::dynamixel_controller__rosidl_typesupport_fastrtps_cpp "${_IMPORT_PREFIX}/lib/libdynamixel_controller__rosidl_typesupport_fastrtps_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
