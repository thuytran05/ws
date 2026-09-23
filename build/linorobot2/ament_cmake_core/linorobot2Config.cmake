# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_linorobot2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED linorobot2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(linorobot2_FOUND FALSE)
  elseif(NOT linorobot2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(linorobot2_FOUND FALSE)
  endif()
  return()
endif()
set(_linorobot2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT linorobot2_FIND_QUIETLY)
  message(STATUS "Found linorobot2: 0.0.0 (${linorobot2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'linorobot2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT linorobot2_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(linorobot2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${linorobot2_DIR}/${_extra}")
endforeach()
