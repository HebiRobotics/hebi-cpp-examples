cmake_minimum_required(VERSION 3.12)
project(hebi_cpp_examples)

SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_STANDARD_REQUIRED ON)

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

find_package(Python 3.6 COMPONENTS Interpreter Development.Module Development.Embed NumPy)

# Due to an issue in Windows and mac with some versions of numpy/distutils/etc, need
# to manually add this here:
set(NUMPY_INCLUDE_DIR ${Python_SITELIB}/numpy/core/include)

# Helper function for adding target with HEBI library/dependency references
function(hebi_add_target TARGET_NAME TARGET_SOURCES)

  if(WIN32)
    add_executable(${TARGET_NAME} ${TARGET_SOURCES} $<TARGET_OBJECTS:_hebic++-obj>)
  else()
    add_executable(${TARGET_NAME} ${TARGET_SOURCES})
  endif()

  ## Add to the "examples" metatarget
  add_dependencies(examples ${TARGET_NAME})

  ## Add include paths, Link and copy libraries
  target_include_directories(${TARGET_NAME} PRIVATE ${ROOT_DIR})

  target_include_directories(${TARGET_NAME} PRIVATE ${Python_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR})
  target_link_libraries(${TARGET_NAME} ${Python_LIBRARIES})
  if(WIN32)
    target_link_libraries(${TARGET_NAME} hebi kernel32)
    target_include_directories(${TARGET_NAME} PRIVATE ${HEBI_DIR}/src ${HEBI_DIR}/include ${HEBI_DIR}/Eigen)
    # For Windows, we copy the .dll file into the binary directory so that we
    # don't have to set the PATH variable.

    set_target_properties(${TARGET_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "$(OutDir)")

    set(LIBHEBI_LOCATION "lib/win_${LIBHEBI_TARGET_ARCHITECTURE}")
    set(HEBI_CPP_LIB_DIRECTORY ${HEBI_DIR}/hebi/${LIBHEBI_LOCATION}/)

    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HEBI_CPP_LIB_DIRECTORY}/hebi.dll"
      $<TARGET_FILE_DIR:${TARGET_NAME}>)
  elseif(UNIX)
    target_link_libraries(${TARGET_NAME} hebi hebic++ m pthread)
  endif()
endfunction()

function(hebi_copy_target_resources TARGET_NAME SOURCE_DIR DEST_DIR)
  # Copy over resource files potentially used by the examples
  add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_directory
                     ${ROOT_DIR}/${SOURCE_DIR} $<TARGET_FILE_DIR:${EX_NAME}>/${DEST_DIR})
endfunction()

function(hebi_make_target_log_directory TARGET_NAME DIR_NAME)
  # Create logs directory in case the example logs data.
  add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E make_directory
                         $<TARGET_FILE_DIR:${TARGET_NAME}>/${DIR_NAME})
endfunction()

# Make one metatarget for all examples
add_custom_target(examples)

# Build X_SERIES Examples
SET(X_SERIES_SOURCES
  ${ROOT_DIR}/basic/actuator/01a_lookup.cpp
  ${ROOT_DIR}/basic/actuator/01b_get_group_single_module.cpp
  ${ROOT_DIR}/basic/actuator/01c_get_group_multiple_modules.cpp
  ${ROOT_DIR}/basic/actuator/02a_feedback_foreground.cpp
  ${ROOT_DIR}/basic/actuator/02b_feedback_background.cpp
  ${ROOT_DIR}/basic/actuator/03a_command_position.cpp
  ${ROOT_DIR}/basic/actuator/03b_command_velocity.cpp
  ${ROOT_DIR}/basic/actuator/03c_command_effort.cpp
  ${ROOT_DIR}/basic/actuator/03d_command_pos_vel.cpp
  ${ROOT_DIR}/basic/actuator/03e_command_pos_vel_effort.cpp
  ${ROOT_DIR}/basic/actuator/03f_command_vel_w_feedback.cpp
  ${ROOT_DIR}/basic/actuator/04a_gains_position_kp.cpp
  ${ROOT_DIR}/basic/actuator/04b_gains_saving_xml.cpp
  ${ROOT_DIR}/basic/actuator/04c_gains_loading_xml.cpp
  ${ROOT_DIR}/basic/actuator/05_trajectory.cpp
  ${ROOT_DIR}/basic/actuator/06a_kinematics_setup.cpp
  ${ROOT_DIR}/basic/actuator/06b_kinematics_fwd_kinematics.cpp
  ${ROOT_DIR}/basic/actuator/06c_kinematics_inv_kinematics.cpp
  ${ROOT_DIR}/basic/actuator/07a_robot_3_dof_arm.cpp
  ${ROOT_DIR}/basic/actuator/07b_robot_6_dof_arm.cpp)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY basic/actuator)
foreach(EXAMPLE ${X_SERIES_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  
  hebi_add_target(${EX_NAME} ${EXAMPLE})
  hebi_copy_target_resources(${EX_NAME} "basic/hrdf" "hrdf")
  hebi_copy_target_resources(${EX_NAME} "basic/gains" "gains")
  hebi_make_target_log_directory(${EX_NAME} "logs")

endforeach()


# Mobile IO Examples
SET(MOBILE_IO_SOURCES
  ${ROOT_DIR}/basic/mobile_io/01a_mobile_io_lookup.cpp
  ${ROOT_DIR}/basic/mobile_io/01b_get_group_single_mobile_io.cpp
  ${ROOT_DIR}/basic/mobile_io/01c_get_group_multiple_mobile_io.cpp
  ${ROOT_DIR}/basic/mobile_io/02a_feedback_mobile_io.cpp
  ${ROOT_DIR}/basic/mobile_io/02b_feedback_mobile_io_w_logging.cpp
  ${ROOT_DIR}/basic/mobile_io/02c_feedback_orientation.cpp
  ${ROOT_DIR}/basic/mobile_io/02d_feedback_full_pose.cpp
  ${ROOT_DIR}/basic/mobile_io/02e_feedback_pose_w_logging.cpp
  ${ROOT_DIR}/basic/mobile_io/02f_feedback_background_mobile_io.cpp
  ${ROOT_DIR}/basic/mobile_io/03a_feedback_io_and_mobile.cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY basic/mobile_io)
foreach(EXAMPLE ${MOBILE_IO_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)

  hebi_add_target(${EX_NAME} ${EXAMPLE})
  hebi_make_target_log_directory(${EX_NAME} "logs")

endforeach()


# IO Board Examples
SET(IO_BOARD_SOURCES
  ${ROOT_DIR}/basic/io_board/01a_io_board_lookup.cpp
  ${ROOT_DIR}/basic/io_board/01b_get_group_single_io_board.cpp
  ${ROOT_DIR}/basic/io_board/01c_get_group_multiple_io_board.cpp
  ${ROOT_DIR}/basic/io_board/02a_feedback_io_board.cpp
  ${ROOT_DIR}/basic/io_board/02b_io_feedback_io_board.cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY basic/io_board)
foreach(EXAMPLE ${IO_BOARD_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)

  hebi_add_target(${EX_NAME} ${EXAMPLE})
  hebi_make_target_log_directory(${EX_NAME} "logs")

endforeach()

# Advanced Examples
SET(ADVANCED_SOURCES
  ${ROOT_DIR}/advanced/lookup/lookup_general_example.cpp
  ${ROOT_DIR}/advanced/lookup/lookup_example.cpp
  ${ROOT_DIR}/advanced/feedback/feedback_async_example.cpp
  ${ROOT_DIR}/advanced/feedback/io_feedback_example.cpp
  ${ROOT_DIR}/advanced/feedback/led_feedback_example.cpp
  ${ROOT_DIR}/advanced/commands/command_control_strategy_example.cpp
  ${ROOT_DIR}/advanced/commands/command_position_example.cpp
  ${ROOT_DIR}/advanced/commands/command_persist_settings_example.cpp
  ${ROOT_DIR}/advanced/commands/command_settings_example.cpp
  ${ROOT_DIR}/advanced/demos/master_slave_async_example.cpp)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY advanced)
foreach(EXAMPLE ${ADVANCED_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  
  hebi_add_target(${EX_NAME} ${EXAMPLE})

endforeach()


# Arm kit Examples
SET(ARM_SOURCES
  ${ROOT_DIR}/kits/arms/ex_gravity_compensation.cpp
  ${ROOT_DIR}/kits/arms/ex_mobile_io_control.cpp
  ${ROOT_DIR}/kits/arms/ex_teach_repeat.cpp
  ${ROOT_DIR}/kits/arms/ex_AR_kit.cpp
  ${ROOT_DIR}/kits/arms/ex_teach_repeat_with_gripper.cpp
  ${ROOT_DIR}/kits/arms/ex_double_arm_teach_repeat.cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY kits/arms)
foreach(EXAMPLE ${ARM_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  
  hebi_add_target(${EX_NAME} ${EXAMPLE})

  # Copy the HRDF files for kits into the build directory
  hebi_copy_target_resources(${EX_NAME} "kits/arms/config" "config")

endforeach()

# Rosie kit example
SET(ROSIE_SOURCES
  ${ROOT_DIR}/kits/rosie/rosie_demo.cpp
  ${ROOT_DIR}/kits/rosie/rosie_demo_utils.cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY kits/rosie)
foreach(EXAMPLE ${ROSIE_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  
  hebi_add_target(${EX_NAME} ${EXAMPLE})
  hebi_copy_target_resources(${EX_NAME} "kits/rosie/config" "config")
  hebi_copy_target_resources(${EX_NAME} "kits/rosie/gains" "gains")
  # Note -- need the arm config files here, too!
  hebi_copy_target_resources(${EX_NAME} "kits/arms/config" "../arms/config")

endforeach()
