# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_px4_multi_uav_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED px4_multi_uav_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(px4_multi_uav_FOUND FALSE)
  elseif(NOT px4_multi_uav_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(px4_multi_uav_FOUND FALSE)
  endif()
  return()
endif()
set(_px4_multi_uav_CONFIG_INCLUDED TRUE)

# output package information
if(NOT px4_multi_uav_FIND_QUIETLY)
  message(STATUS "Found px4_multi_uav: 0.0.0 (${px4_multi_uav_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'px4_multi_uav' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${px4_multi_uav_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(px4_multi_uav_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${px4_multi_uav_DIR}/${_extra}")
endforeach()
