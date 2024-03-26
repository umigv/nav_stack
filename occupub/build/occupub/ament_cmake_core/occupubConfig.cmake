# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_occupub_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED occupub_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(occupub_FOUND FALSE)
  elseif(NOT occupub_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(occupub_FOUND FALSE)
  endif()
  return()
endif()
set(_occupub_CONFIG_INCLUDED TRUE)

# output package information
if(NOT occupub_FIND_QUIETLY)
  message(STATUS "Found occupub: 0.0.0 (${occupub_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'occupub' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${occupub_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(occupub_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${occupub_DIR}/${_extra}")
endforeach()
