#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "srdfdom::srdfdom" for configuration "Debug"
set_property(TARGET srdfdom::srdfdom APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(srdfdom::srdfdom PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libsrdfdom.so.2.0.7"
  IMPORTED_SONAME_DEBUG "libsrdfdom.so.2.0.7"
  )

list(APPEND _cmake_import_check_targets srdfdom::srdfdom )
list(APPEND _cmake_import_check_files_for_srdfdom::srdfdom "${_IMPORT_PREFIX}/lib/libsrdfdom.so.2.0.7" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
