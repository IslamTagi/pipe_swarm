# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pipe_swarm_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pipe_swarm_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pipe_swarm_FOUND FALSE)
  elseif(NOT pipe_swarm_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pipe_swarm_FOUND FALSE)
  endif()
  return()
endif()
set(_pipe_swarm_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pipe_swarm_FIND_QUIETLY)
  message(STATUS "Found pipe_swarm: 0.0.0 (${pipe_swarm_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pipe_swarm' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pipe_swarm_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pipe_swarm_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pipe_swarm_DIR}/${_extra}")
endforeach()
