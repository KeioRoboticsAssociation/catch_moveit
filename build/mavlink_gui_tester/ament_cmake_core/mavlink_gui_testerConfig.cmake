# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mavlink_gui_tester_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mavlink_gui_tester_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mavlink_gui_tester_FOUND FALSE)
  elseif(NOT mavlink_gui_tester_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mavlink_gui_tester_FOUND FALSE)
  endif()
  return()
endif()
set(_mavlink_gui_tester_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mavlink_gui_tester_FIND_QUIETLY)
  message(STATUS "Found mavlink_gui_tester: 0.0.0 (${mavlink_gui_tester_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mavlink_gui_tester' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mavlink_gui_tester_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mavlink_gui_tester_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mavlink_gui_tester_DIR}/${_extra}")
endforeach()
