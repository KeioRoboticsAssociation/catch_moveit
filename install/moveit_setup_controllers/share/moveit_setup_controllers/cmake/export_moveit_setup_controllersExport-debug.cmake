#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "moveit_setup_controllers::moveit_setup_controllers" for configuration "Debug"
set_property(TARGET moveit_setup_controllers::moveit_setup_controllers APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(moveit_setup_controllers::moveit_setup_controllers PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmoveit_setup_controllers.so"
  IMPORTED_SONAME_DEBUG "libmoveit_setup_controllers.so"
  )

list(APPEND _cmake_import_check_targets moveit_setup_controllers::moveit_setup_controllers )
list(APPEND _cmake_import_check_files_for_moveit_setup_controllers::moveit_setup_controllers "${_IMPORT_PREFIX}/lib/libmoveit_setup_controllers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
