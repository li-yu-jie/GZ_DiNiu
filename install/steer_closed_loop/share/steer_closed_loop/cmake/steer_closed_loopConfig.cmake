# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_steer_closed_loop_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED steer_closed_loop_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(steer_closed_loop_FOUND FALSE)
  elseif(NOT steer_closed_loop_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(steer_closed_loop_FOUND FALSE)
  endif()
  return()
endif()
set(_steer_closed_loop_CONFIG_INCLUDED TRUE)

# output package information
if(NOT steer_closed_loop_FIND_QUIETLY)
  message(STATUS "Found steer_closed_loop: 0.0.1 (${steer_closed_loop_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'steer_closed_loop' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${steer_closed_loop_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(steer_closed_loop_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${steer_closed_loop_DIR}/${_extra}")
endforeach()
