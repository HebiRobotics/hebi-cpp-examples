cmake_minimum_required(VERSION 2.6)
project(hebi_input_api)

set(HEBI_INPUT_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/event_handler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/joystick.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/joystick_mapper.cpp)

add_library(hebi_input SHARED ${HEBI_INPUT_SOURCES})

find_package(SDL2 REQUIRED)

target_include_directories(hebi_input PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${SDL2_INCLUDE_DIRS})
target_link_libraries(hebi_input SDL2)
set_target_properties(hebi_input PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES)
