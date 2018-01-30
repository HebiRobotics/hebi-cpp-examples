cmake_minimum_required(VERSION 2.6)
project(hebi_cpp_examples)

################
# Import library and add dependencies

# You can choose which architecture to target by passing `-DARCH=<arch>` as an argument.
# See `hebi_platform.cmake` to see valid choices. Note that the valid choices depends
# on the platform on which this machine is running.
if(NOT DEFINED ARCH)
  set(ARCH "native")
endif()

SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_STANDARD_REQUIRED ON)

include(hebi_platform.cmake)

if(HEBI_HOST_LINUX)
  set(HEBI_C_LIB_SUBDIR "lib/linux_${HEBI_TARGET_ARCH}")
elseif(HEBI_HOST_WINDOWS)
  set(HEBI_C_LIB_SUBDIR "lib/win_${HEBI_TARGET_ARCH}")
elseif(HEBI_HOST_OSX)
  set(HEBI_C_LIB_SUBDIR "lib/osx_${HEBI_TARGET_ARCH}")
else()
  message(FATAL_ERROR "Unkown host platform")
endif()

set (ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set (HEBI_DIR ${ROOT_DIR}/hebi)

################
# Setup project

set(HEBI_CPP_INCLUDE_DIRECTORIES
  ${HEBI_DIR}/src
  ${HEBI_DIR}/include
  ${HEBI_DIR}/include/Eigen
)

set(HEBI_CPP_SOURCES
  ${HEBI_DIR}/src/command.cpp
  ${HEBI_DIR}/src/feedback.cpp
  ${HEBI_DIR}/src/group_command.cpp
  ${HEBI_DIR}/src/group.cpp
  ${HEBI_DIR}/src/group_feedback.cpp
  ${HEBI_DIR}/src/group_info.cpp
  ${HEBI_DIR}/src/info.cpp
  ${HEBI_DIR}/src/robot_model.cpp
  ${HEBI_DIR}/src/lookup.cpp
  ${HEBI_DIR}/src/mac_address.cpp
  ${HEBI_DIR}/src/trajectory.cpp
  ${HEBI_DIR}/src/log_file.cpp
)

set(HEBI_CPP_LINK_DIRECTORIES ${HEBI_DIR}/${HEBI_C_LIB_SUBDIR})
set(HEBI_CPP_LIB_DIRECTORY ${HEBI_DIR}/${HEBI_C_LIB_SUBDIR})

include_directories(${HEBI_CPP_INCLUDE_DIRECTORIES})
link_directories(${HEBI_CPP_LINK_DIRECTORIES})

# Packaging sources into object library allows reuse if you have multiple targets
add_library(hebi_cpp_files OBJECT ${HEBI_CPP_SOURCES})

################
# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Build all examples:
SET(EXAMPLES_SOURCES

  ${ROOT_DIR}/basic/01_lookup.cpp
  ${ROOT_DIR}/basic/02_feedback.cpp
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
  ${ROOT_DIR}/advanced/commands/command_position_example.cpp
  ${ROOT_DIR}/advanced/commands/command_persist_settings_example.cpp
  ${ROOT_DIR}/advanced/commands/command_settings_example.cpp

  ${ROOT_DIR}/advanced/demos/master_slave_async_example.cpp
  ${ROOT_DIR}/advanced/demos/arm_teach_repeat.cpp

)

# Make one metatarget for all examples
add_custom_target(examples) 

foreach (EXAMPLE ${EXAMPLES_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  add_executable(${EX_NAME} ${EXAMPLE} $<TARGET_OBJECTS:hebi_cpp_files>)

  # Add this example to metatarget
  add_dependencies(examples ${EX_NAME})

  # Add ultra-conservative warnings.
  if(CMAKE_COMPILER_IS_GNUCC OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set_property(TARGET ${EX_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS -Wall)
  endif()

  # Platform-specific linkage
  if(WIN32)
    
    target_link_libraries(${EX_NAME} hebi kernel32)
    # For Windows, we copy the .dll file into the binary directory so that we
    # don't have to set the PATH variable.
    add_custom_command(TARGET ${EX_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HEBI_CPP_LIB_DIRECTORY}/hebi.dll"
      $<TARGET_FILE_DIR:${EX_NAME}>
    )

  elseif(UNIX)
    target_link_libraries(${EX_NAME} hebi m pthread)
  endif()

endforeach (EXAMPLE ${EXAMPLES_SOURCES})
