#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "moveit_setup_srdf_plugins::moveit_setup_srdf_plugins" for configuration "Debug"
set_property(TARGET moveit_setup_srdf_plugins::moveit_setup_srdf_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(moveit_setup_srdf_plugins::moveit_setup_srdf_plugins PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmoveit_setup_srdf_plugins.so"
  IMPORTED_SONAME_DEBUG "libmoveit_setup_srdf_plugins.so"
  )

list(APPEND _cmake_import_check_targets moveit_setup_srdf_plugins::moveit_setup_srdf_plugins )
list(APPEND _cmake_import_check_files_for_moveit_setup_srdf_plugins::moveit_setup_srdf_plugins "${_IMPORT_PREFIX}/lib/libmoveit_setup_srdf_plugins.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
