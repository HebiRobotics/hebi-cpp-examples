cmake_minimum_required(VERSION 3.0)
project(hebi_cpp_examples)

set (ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set (HEBI_DIR ${ROOT_DIR}/hebi)

# C++ API has not been downloaded - download it
if(NOT EXISTS ${HEBI_DIR})
  message(WARNING "ERROR: C++ API not found -- run the `scripts/download_depends.py` script or manually extract")
  message(FATAL_ERROR "you need to put the C++ api in the hebi folder")
endif()

add_subdirectory(${HEBI_DIR} ${CMAKE_CURRENT_BINARY_DIR}/hebi-cpp-${CMAKE_BUILD_TYPE})

################
# Setup project

set(HEBI_CPP_LINK_DIRECTORIES ${HEBI_DIR}/${HEBI_C_LIB_SUBDIR})
set(HEBI_CPP_LIB_DIRECTORY ${HEBI_DIR}/${HEBI_C_LIB_SUBDIR})

################
# Build examples

# Build all examples:
SET(EXAMPLES_SOURCES

  ${ROOT_DIR}/basic/01_lookup.cpp
  ${ROOT_DIR}/basic/02a_feedback_foreground.cpp
  ${ROOT_DIR}/basic/02b_feedback_background.cpp
  ${ROOT_DIR}/basic/03_command.cpp
  ${ROOT_DIR}/basic/04_kinematics.cpp
  ${ROOT_DIR}/basic/05_ik.cpp
  ${ROOT_DIR}/basic/06_trajectory.cpp

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

  ${ROOT_DIR}/kits/arm/gravity_compensation.cpp
  ${ROOT_DIR}/kits/arm/teach_repeat.cpp
)

# Make one metatarget for all examples
add_custom_target(examples)

foreach (EXAMPLE ${EXAMPLES_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  add_executable(${EX_NAME} ${EXAMPLE})

  # Add this example to metatarget
  add_dependencies(examples ${EX_NAME})

  # Add ultra-conservative warnings.
  if(CMAKE_COMPILER_IS_GNUCC OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set_property(TARGET ${EX_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS -Wall)
  endif()

  target_include_directories(${EX_NAME} PRIVATE ${ROOT_DIR})

  # Platform-specific linkage
  if(WIN32)
    
    target_link_libraries(${EX_NAME} hebi hebic++ kernel32)
    # For Windows, we copy the .dll file into the binary directory so that we
    # don't have to set the PATH variable.
    add_custom_command(TARGET ${EX_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HEBI_CPP_LIB_DIRECTORY}/hebi.dll"
      $<TARGET_FILE_DIR:${EX_NAME}>
    )

  elseif(UNIX)
    target_link_libraries(${EX_NAME} hebi hebic++ m pthread)
  endif()

endforeach (EXAMPLE ${EXAMPLES_SOURCES})
