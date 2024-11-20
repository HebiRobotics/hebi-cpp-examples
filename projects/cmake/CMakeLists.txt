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

# Make one metatarget for all examples
add_custom_target(examples)

# Build X_SERIES Examples
SET(X_SERIES_SOURCES
  ${ROOT_DIR}/basic/x_series_actuator/01a_lookup.cpp
  ${ROOT_DIR}/basic/x_series_actuator/01b_get_group_single_module.cpp
  ${ROOT_DIR}/basic/x_series_actuator/01c_get_group_multiple_modules.cpp
  ${ROOT_DIR}/basic/x_series_actuator/02a_feedback_foreground.cpp
  ${ROOT_DIR}/basic/x_series_actuator/02b_feedback_background.cpp
  ${ROOT_DIR}/basic/x_series_actuator/03a_command_position.cpp
  ${ROOT_DIR}/basic/x_series_actuator/03b_command_velocity.cpp
  ${ROOT_DIR}/basic/x_series_actuator/03c_command_effort.cpp
  ${ROOT_DIR}/basic/x_series_actuator/03d_command_pos_vel.cpp
  ${ROOT_DIR}/basic/x_series_actuator/03e_command_pos_vel_effort.cpp
  ${ROOT_DIR}/basic/x_series_actuator/03f_command_vel_w_feedback.cpp
  ${ROOT_DIR}/basic/x_series_actuator/04a_gains_position_kp.cpp
  ${ROOT_DIR}/basic/x_series_actuator/04b_gains_saving_xml.cpp
  ${ROOT_DIR}/basic/x_series_actuator/04c_gains_loading_xml.cpp
  ${ROOT_DIR}/basic/x_series_actuator/05_trajectory.cpp
  ${ROOT_DIR}/basic/x_series_actuator/06a_kinematics_setup.cpp
  ${ROOT_DIR}/basic/x_series_actuator/06b_kinematics_fwd_kinematics.cpp
  ${ROOT_DIR}/basic/x_series_actuator/06c_kinematics_inv_kinematics.cpp
  ${ROOT_DIR}/basic/x_series_actuator/07a_robot_3_dof_arm.cpp
  ${ROOT_DIR}/basic/x_series_actuator/07b_robot_6_dof_arm.cpp)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY basic/x_series_actuator)
foreach (EXAMPLE ${X_SERIES_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  
  if(WIN32)
    add_executable(${EX_NAME} ${EXAMPLE} $<TARGET_OBJECTS:_hebic++-obj>)
  else()
    add_executable(${EX_NAME} ${EXAMPLE})

  # Copy over resource files potentially used by the examples
  add_custom_command(TARGET ${EX_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_directory
                         ${ROOT_DIR}/basic/hrdf $<TARGET_FILE_DIR:${EX_NAME}>/hrdf)
  add_custom_command(TARGET ${EX_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_directory
                         ${ROOT_DIR}/basic/gains $<TARGET_FILE_DIR:${EX_NAME}>/gains)

  # Create logs directory in case the example logs data.
  add_custom_command(TARGET ${EX_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E make_directory
                         $<TARGET_FILE_DIR:${EX_NAME}>/logs)

  endif()
  add_dependencies(examples ${EX_NAME})
  target_include_directories(${EX_NAME} PRIVATE ${ROOT_DIR})

  target_include_directories(${EX_NAME} PRIVATE ${Python_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR})
  target_link_libraries(${EX_NAME} ${Python_LIBRARIES})
  if(WIN32)
    target_link_libraries(${EX_NAME} hebi kernel32)
    target_include_directories(${EX_NAME} PRIVATE ${HEBI_DIR}/src ${HEBI_DIR}/include ${HEBI_DIR}/Eigen)
    # For Windows, we copy the .dll file into the binary directory so that we
    # don't have to set the PATH variable.

    set(LIBHEBI_LOCATION "lib/win_${LIBHEBI_TARGET_ARCHITECTURE}")
    set(HEBI_CPP_LIB_DIRECTORY ${HEBI_DIR}/hebi/${LIBHEBI_LOCATION}/)

    add_custom_command(TARGET ${EX_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HEBI_CPP_LIB_DIRECTORY}/hebi.dll"
      $<TARGET_FILE_DIR:${EX_NAME}>)
  elseif(UNIX)
    target_link_libraries(${EX_NAME} hebi hebic++ m pthread)
  endif()

endforeach (EXAMPLE ${X_SERIES_SOURCES})




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
foreach (EXAMPLE ${MOBILE_IO_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  
  if(WIN32)
    add_executable(${EX_NAME} ${EXAMPLE} $<TARGET_OBJECTS:_hebic++-obj>)
  else()
    add_executable(${EX_NAME} ${EXAMPLE})

  # Create logs directory in case the example logs data.
  add_custom_command(TARGET ${EX_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E make_directory
                         $<TARGET_FILE_DIR:${EX_NAME}>/logs)
  endif()
  add_dependencies(examples ${EX_NAME})
  target_include_directories(${EX_NAME} PRIVATE ${ROOT_DIR})

  target_include_directories(${EX_NAME} PRIVATE ${Python_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR})
  target_link_libraries(${EX_NAME} ${Python_LIBRARIES})
  if(WIN32)
    target_link_libraries(${EX_NAME} hebi kernel32)
    target_include_directories(${EX_NAME} PRIVATE ${HEBI_DIR}/src ${HEBI_DIR}/include ${HEBI_DIR}/Eigen)
    # For Windows, we copy the .dll file into the binary directory so that we
    # don't have to set the PATH variable.

    set(LIBHEBI_LOCATION "lib/win_${LIBHEBI_TARGET_ARCHITECTURE}")
    set(HEBI_CPP_LIB_DIRECTORY ${HEBI_DIR}/hebi/${LIBHEBI_LOCATION}/)

    add_custom_command(TARGET ${EX_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HEBI_CPP_LIB_DIRECTORY}/hebi.dll"
      $<TARGET_FILE_DIR:${EX_NAME}>)
  elseif(UNIX)
    target_link_libraries(${EX_NAME} hebi hebic++ m pthread)
  endif()

endforeach (EXAMPLE ${MOBILE_IO_SOURCES})




# IO Board Examples
SET(IO_BOARD_SOURCES
  ${ROOT_DIR}/basic/io_board/01a_io_board_lookup.cpp
  ${ROOT_DIR}/basic/io_board/01b_get_group_single_io_board.cpp
  ${ROOT_DIR}/basic/io_board/01c_get_group_multiple_io_board.cpp
  ${ROOT_DIR}/basic/io_board/02a_feedback_io_board.cpp
  ${ROOT_DIR}/basic/io_board/02b_io_feedback_io_board.cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY basic/io_board)
foreach (EXAMPLE ${IO_BOARD_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  
  if(WIN32)
    add_executable(${EX_NAME} ${EXAMPLE} $<TARGET_OBJECTS:_hebic++-obj>)
  else()
    add_executable(${EX_NAME} ${EXAMPLE})

  # Create logs directory in case the example logs data.
  add_custom_command(TARGET ${EX_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E make_directory
                         $<TARGET_FILE_DIR:${EX_NAME}>/logs)
  endif()
  add_dependencies(examples ${EX_NAME})
  target_include_directories(${EX_NAME} PRIVATE ${ROOT_DIR})

  target_include_directories(${EX_NAME} PRIVATE ${Python_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR})
  target_link_libraries(${EX_NAME} ${Python_LIBRARIES})
  if(WIN32)
    target_link_libraries(${EX_NAME} hebi kernel32)
    target_include_directories(${EX_NAME} PRIVATE ${HEBI_DIR}/src ${HEBI_DIR}/include ${HEBI_DIR}/Eigen)
    # For Windows, we copy the .dll file into the binary directory so that we
    # don't have to set the PATH variable.

    set(LIBHEBI_LOCATION "lib/win_${LIBHEBI_TARGET_ARCHITECTURE}")
    set(HEBI_CPP_LIB_DIRECTORY ${HEBI_DIR}/hebi/${LIBHEBI_LOCATION}/)

    add_custom_command(TARGET ${EX_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HEBI_CPP_LIB_DIRECTORY}/hebi.dll"
      $<TARGET_FILE_DIR:${EX_NAME}>)
  elseif(UNIX)
    target_link_libraries(${EX_NAME} hebi hebic++ m pthread)
  endif()

endforeach (EXAMPLE ${IO_BOARD_SOURCES})




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
foreach (EXAMPLE ${ADVANCED_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  
  if(WIN32)
    add_executable(${EX_NAME} ${EXAMPLE} $<TARGET_OBJECTS:_hebic++-obj>)
  else()
    add_executable(${EX_NAME} ${EXAMPLE})
  endif()
  add_dependencies(examples ${EX_NAME})
  target_include_directories(${EX_NAME} PRIVATE ${ROOT_DIR})

  target_include_directories(${EX_NAME} PRIVATE ${Python_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR})
  target_link_libraries(${EX_NAME} ${Python_LIBRARIES})
  if(WIN32)
    target_link_libraries(${EX_NAME} hebi kernel32)
    target_include_directories(${EX_NAME} PRIVATE ${HEBI_DIR}/src ${HEBI_DIR}/include ${HEBI_DIR}/Eigen)
    # For Windows, we copy the .dll file into the binary directory so that we
    # don't have to set the PATH variable.

    set(LIBHEBI_LOCATION "lib/win_${LIBHEBI_TARGET_ARCHITECTURE}")
    set(HEBI_CPP_LIB_DIRECTORY ${HEBI_DIR}/hebi/${LIBHEBI_LOCATION}/)

    add_custom_command(TARGET ${EX_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HEBI_CPP_LIB_DIRECTORY}/hebi.dll"
      $<TARGET_FILE_DIR:${EX_NAME}>)
  elseif(UNIX)
    target_link_libraries(${EX_NAME} hebi hebic++ m pthread)
  endif()

endforeach (EXAMPLE ${ADVANCED_SOURCES})




# Kit Examples
SET(ARM_SOURCES

  ${ROOT_DIR}/kits/arm/ex_gravity_compensation.cpp
  ${ROOT_DIR}/kits/arm/ex_mobile_io_control.cpp
  ${ROOT_DIR}/kits/arm/ex_teach_repeat.cpp
  ${ROOT_DIR}/kits/arm/ex_AR_kit.cpp
  ${ROOT_DIR}/kits/arm/ex_teach_repeat_with_gripper.cpp
  ${ROOT_DIR}/kits/arm/ex_double_arm_teach_repeat.cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY kits/arm)
foreach (EXAMPLE ${ARM_SOURCES})

  # The target for the individual example is based on the filename
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  
  if(WIN32)
    add_executable(${EX_NAME} ${EXAMPLE} $<TARGET_OBJECTS:_hebic++-obj>)
  else()
    add_executable(${EX_NAME} ${EXAMPLE})
  endif()
  add_dependencies(examples ${EX_NAME})
  target_include_directories(${EX_NAME} PRIVATE ${ROOT_DIR})

  target_include_directories(${EX_NAME} PRIVATE ${Python_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR})
  target_link_libraries(${EX_NAME} ${Python_LIBRARIES})
  if(WIN32)
    target_link_libraries(${EX_NAME} hebi kernel32)
    target_include_directories(${EX_NAME} PRIVATE ${HEBI_DIR}/src ${HEBI_DIR}/include ${HEBI_DIR}/Eigen)
    # For Windows, we copy the .dll file into the binary directory so that we
    # don't have to set the PATH variable.

    set(LIBHEBI_LOCATION "lib/win_${LIBHEBI_TARGET_ARCHITECTURE}")
    set(HEBI_CPP_LIB_DIRECTORY ${HEBI_DIR}/hebi/${LIBHEBI_LOCATION}/)

    add_custom_command(TARGET ${EX_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HEBI_CPP_LIB_DIRECTORY}/hebi.dll"
      $<TARGET_FILE_DIR:${EX_NAME}>)
  elseif(UNIX)
    target_link_libraries(${EX_NAME} hebi hebic++ m pthread)
  endif()

endforeach (EXAMPLE ${KIT_SOURCES})


# Copy the HRDF files for kits into the build directory
file(COPY ${ROOT_DIR}/kits/arm/config DESTINATION ${CMAKE_BINARY_DIR}/kits/arm)
