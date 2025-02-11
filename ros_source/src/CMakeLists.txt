# Use at least CMake 3.0.2 (ROS Noetic supports CMake ≥3)
cmake_minimum_required(VERSION 3.0.2)

# Define your project name
project(AllegroHand)

#--- Begin catkin workspace setup -------------------------------------------
# Tell catkin that this is a top-level workspace
set(CATKIN_TOPLEVEL TRUE)

# Try to locate catkin inside the workspace using the helper script
set(_cmd "catkin_find_pkg" "catkin" "${CMAKE_SOURCE_DIR}")
execute_process(
  COMMAND ${_cmd}
  RESULT_VARIABLE _res
  OUTPUT_VARIABLE _out
  ERROR_VARIABLE _err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)

if(NOT _res EQUAL 0 AND NOT _res EQUAL 2)
  # An error occurred while searching for catkin
  string(REPLACE ";" " " _cmd_str "${_cmd}")
  message(FATAL_ERROR "Search for 'catkin' in workspace failed (${_cmd_str}): ${_err}")
endif()

if(_res EQUAL 0)
  # If found in the workspace, include the catkin extras.
  set(catkin_EXTRAS_DIR "${CMAKE_SOURCE_DIR}/${_out}/cmake")
  include(${catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)
  add_subdirectory("${_out}")
else()
  # Otherwise, look for catkin in the CMAKE_PREFIX_PATH (set externally or via ROS setup)
  if(NOT DEFINED CMAKE_PREFIX_PATH)
    if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      if(NOT WIN32)
        string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      else()
        set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      endif()
    endif()
  endif()

  # Look through the CMAKE_PREFIX_PATH for catkin workspaces.
  set(catkin_search_path "")
  foreach(path ${CMAKE_PREFIX_PATH})
    if(EXISTS "${path}/.catkin")
      list(FIND catkin_search_path ${path} _index)
      if(_index EQUAL -1)
        list(APPEND catkin_search_path ${path})
      endif()
    endif()
  endforeach()

  # Find catkin in those workspaces.
  set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
  find_package(catkin QUIET
    NO_POLICY_SCOPE
    PATHS ${catkin_search_path}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  unset(CATKIN_TOPLEVEL_FIND_PACKAGE)

  if(NOT catkin_FOUND)
    message(FATAL_ERROR "find_package(catkin) failed. Catkin was neither found in the workspace nor in the CMAKE_PREFIX_PATH. Did you source your ROS setup.sh?")
  endif()
endif()

# Now perform any additional workspace setup provided by catkin.
catkin_workspace()