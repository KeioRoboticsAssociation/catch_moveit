#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dynamixel_sdk::dynamixel_sdk" for configuration "Release"
set_property(TARGET dynamixel_sdk::dynamixel_sdk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(dynamixel_sdk::dynamixel_sdk PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdynamixel_sdk.so"
  IMPORTED_SONAME_RELEASE "libdynamixel_sdk.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS dynamixel_sdk::dynamixel_sdk )
list(APPEND _IMPORT_CHECK_FILES_FOR_dynamixel_sdk::dynamixel_sdk "${_IMPORT_PREFIX}/lib/libdynamixel_sdk.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
