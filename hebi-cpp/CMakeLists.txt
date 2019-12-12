cmake_minimum_required(VERSION 3.0)
project(hebi_cpp_api VERSION 3.1.0)

# If you want to include the C API from an out of source tree location,
# pass `-DHEBI_C_DIR=<dir>` to cmake when generating this project
if (NOT DEFINED HEBI_C_DIR)
  set(HEBI_C_DIR hebi)
endif()

get_filename_component(HEBI_C_DIR ${HEBI_C_DIR} ABSOLUTE)
add_subdirectory(${HEBI_C_DIR} hebi-c-build-dir)

set(hebi_cpp_src
  src/mac_address.cpp
  src/command.cpp
  src/feedback.cpp
  src/info.cpp
  src/message_helpers.cpp
  src/group_command.cpp
  src/group_feedback.cpp
  src/group_info.cpp
  src/group.cpp
  src/log_file.cpp
  src/lookup.cpp
  src/trajectory.cpp
  src/robot_model.cpp
  src/version.cpp
  src/arm/arm.cpp
  src/arm/kinematics_helper.cpp)

set(hebi_cpp_util
  src/util/mobile_io.cpp)

add_library(_hebic++-obj OBJECT ${hebi_cpp_src} ${hebi_cpp_util})
add_library(hebic++ SHARED $<TARGET_OBJECTS:_hebic++-obj>)
add_library(hebic++-static STATIC $<TARGET_OBJECTS:_hebic++-obj>)

target_link_libraries(hebic++ PUBLIC hebi)
target_link_libraries(hebic++-static PUBLIC hebi)

target_include_directories(_hebic++-obj
  PUBLIC src util include Eigen
  $<TARGET_PROPERTY:hebi,INTERFACE_INCLUDE_DIRECTORIES>)
target_include_directories(hebic++ INTERFACE $<TARGET_PROPERTY:_hebic++-obj,INTERFACE_INCLUDE_DIRECTORIES>)
target_include_directories(hebic++-static INTERFACE $<TARGET_PROPERTY:_hebic++-obj,INTERFACE_INCLUDE_DIRECTORIES>)

set_target_properties(_hebic++-obj hebic++ hebic++-static PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  POSITION_INDEPENDENT_CODE ON
  VERSION ${hebi_cpp_api_VERSION})
