cmake_minimum_required(VERSION 3.0)
project(hebi_cpp_examples)

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
get_filename_component(ROOT_DIR "${ROOT_DIR}" REALPATH)

# `HEBI_CPP_DIR` can be used to point to an already downloaded C++ API.
# This is useful for environments that would prefer not to download files
# from the internet (or even don't have internet access).
if(NOT HEBI_CPP_DIR)
  set(HEBI_DIR ${ROOT_DIR}/hebi-cpp)
  include(DownloadHebiCpp.cmake)
else()
  set(HEBI_DIR ${HEBI_CPP_DIR})
endif()

set(hebi_cpp_build_dir hebi-cpp-${CMAKE_BUILD_TYPE})
string(TOLOWER ${hebi_cpp_build_dir} hebi_cpp_build_dir)
get_filename_component(hebi_cpp_build_dir "${CMAKE_CURRENT_BINARY_DIR}/${hebi_cpp_build_dir}" REALPATH)

add_subdirectory(${HEBI_DIR} ${hebi_cpp_build_dir})

set(HEBI_CPP_LIB_DIRECTORY ${HEBI_DIR}/${HEBI_C_LIB_SUBDIR})

set(EXAMPLES_SOURCES
# Basic Examples
  ${ROOT_DIR}/basic/01_lookup.cpp
  ${ROOT_DIR}/basic/02a_feedback_foreground.cpp
  ${ROOT_DIR}/basic/02b_feedback_background.cpp
  ${ROOT_DIR}/basic/03_command.cpp
  ${ROOT_DIR}/basic/04_kinematics.cpp
  ${ROOT_DIR}/basic/05_ik.cpp
  ${ROOT_DIR}/basic/06_trajectory.cpp
# Advanced examples
  ${ROOT_DIR}/advanced/lookup/lookup_general_example.cpp
  ${ROOT_DIR}/advanced/lookup/lookup_example.cpp
  ${ROOT_DIR}/advanced/feedback/feedback_async_example.cpp
  ${ROOT_DIR}/advanced/feedback/io_feedback_example.cpp
  ${ROOT_DIR}/advanced/feedback/led_feedback_example.cpp
  ${ROOT_DIR}/advanced/commands/command_control_strategy_example.cpp
  ${ROOT_DIR}/advanced/commands/command_gains.cpp
  ${ROOT_DIR}/advanced/commands/command_position_example.cpp
  ${ROOT_DIR}/advanced/commands/command_persist_settings_example.cpp
  ${ROOT_DIR}/advanced/commands/command_settings_example.cpp
  ${ROOT_DIR}/advanced/demos/master_slave_async_example.cpp
# Kits
  ${ROOT_DIR}/kits/arm/gravity_compensation.cpp
  ${ROOT_DIR}/kits/arm/teach_repeat.cpp)

# Make one metatarget for all examples
add_custom_target(examples)

foreach (EXAMPLE ${EXAMPLES_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  if(WIN32)
    add_executable(${EX_NAME} ${EXAMPLE} $<TARGET_OBJECTS:_hebic++-obj>)
  else()
    add_executable(${EX_NAME} ${EXAMPLE})
  endif()
  add_dependencies(examples ${EX_NAME})
  target_include_directories(${EX_NAME} PRIVATE ${ROOT_DIR})

  if(WIN32)
    target_link_libraries(${EX_NAME} hebi kernel32)
    # For Windows, we copy the .dll file into the binary directory so that we
    # don't have to set the PATH variable.
    add_custom_command(TARGET ${EX_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HEBI_CPP_LIB_DIRECTORY}/hebi.dll"
      $<TARGET_FILE_DIR:${EX_NAME}>)
  elseif(UNIX)
    target_link_libraries(${EX_NAME} hebi hebic++ m pthread)
  endif()

endforeach (EXAMPLE ${EXAMPLES_SOURCES})
