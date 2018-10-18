cmake_minimum_required (VERSION 3.1)
project (hexapod)

SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_STANDARD_REQUIRED ON)
SET (CMAKE_CXX_FLAGS "-O3 -g")
SET (SONAME "hebi")

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Find Qt libraries
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets NO_CMAKE_PATH)
# NOTE: to add support for 3D Qt components, replace the above with the
# following line
#find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets 3DCore 3DExtras 3DRender 3DInput NO_CMAKE_PATH)

find_package(Doxygen)

# Get the C++ API directory:
set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
get_filename_component(ROOT_DIR "${ROOT_DIR}" REALPATH)
# `HEBI_CPP_DIR` can be used to point to an already downloaded C++ API.
# This is useful for environments that would prefer not to download files
# from the internet (or even don't have internet access).
if(NOT HEBI_CPP_DIR)
  set(HEBI_DIR ${ROOT_DIR}/hebi-cpp)
  include(../../projects/cmake/DownloadHebiCpp.cmake)
else()
  set(HEBI_DIR ${HEBI_CPP_DIR})
endif()

set(hebi_cpp_build_dir hebi-cpp-${CMAKE_BUILD_TYPE})
string(TOLOWER ${hebi_cpp_build_dir} hebi_cpp_build_dir)
get_filename_component(hebi_cpp_build_dir "${CMAKE_CURRENT_BINARY_DIR}/${hebi_cpp_build_dir}" REALPATH)
add_subdirectory(${HEBI_DIR} ${hebi_cpp_build_dir})

include_directories (
  ${CMAKE_CURRENT_SOURCE_DIR}/src          # Our source files
  ${HEBI_DIR}/src ${HEBI_DIR}/hebi/include ${HEBI_DIR}/Eigen)
link_directories (
  ${HEBI_CPP_LINK_DIRECTORIES})

add_library(hexapod_core OBJECT
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot/leg.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot/hexapod.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot/step.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot/hexapod_parameters.cpp
)

SET(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/display/hexapod_view_2d.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/input/input_manager_mobile_io.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/xml_util/pugixml.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/xml_util/xml_helpers.cpp
)

SET(PARAMETER_GUI_2D_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parameter_gui_2d.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/param_display/hexapod_parameter_view_2d.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/param_display/hexapod_parameter_gui.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/display/hexapod_view_2d.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/input/input_manager_mobile_io.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/xml_util/pugixml.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/xml_util/xml_helpers.cpp
)

SET(PARAMETER_GUI_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parameter_gui.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/param_display/hexapod_parameter_view.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/param_display/hexapod_parameter_gui.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/input/input_manager_mobile_io.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/xml_util/pugixml.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/xml_util/xml_helpers.cpp
)

SET(INPUT_TEST_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/input_test.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/input/input_manager_mobile_io.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/xml_util/pugixml.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/xml_util/xml_helpers.cpp
)

add_executable(hexapod_control ${SOURCES} $<TARGET_OBJECTS:hexapod_core>)
qt5_use_modules(hexapod_control Core Gui Widgets)

if (WIN32) 
target_link_libraries( hexapod_control ${SONAME} kernel32 ) # kernel32 for sleep commands.
else()
target_link_libraries( hexapod_control ${SONAME} hebi hebic++ m pthread)
endif()

# Add ultra-conservative warnings.
if ( CMAKE_COMPILER_IS_GNUCC )
  set_property( TARGET hexapod_control APPEND_STRING PROPERTY COMPILE_FLAGS " -Wall " )
endif ( CMAKE_COMPILER_IS_GNUCC )
# For Windows, we copy the .dll file into the binary directory so that we don't have to set the PATH variable.
if (WIN32)
add_custom_command( TARGET hexapod_control POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  "${PROJECT_SOURCE_DIR}/lib/${SONAME}.dll"
  $<TARGET_FILE_DIR:hexapod_control>)
endif()

add_custom_command(TARGET hexapod_control POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_CURRENT_SOURCE_DIR}/resources/ $<TARGET_FILE_DIR:hexapod_control>/)

# Create logs directory in case the example logs data.
add_custom_command(TARGET hexapod_control POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory
                       $<TARGET_FILE_DIR:hexapod_control>/logs)

# Build all examples:
# Parameter setting GUI
# NOTE: due to an issue with the Qt53DRender ubuntu 18.04 package, we have
# temporarily disabled the 3D parameter GUI.  To reenable, add support for 3D Qt
# components in the above `find_package` scripts, and uncomment the following
# lines.
#add_executable(parameter_gui ${PARAMETER_GUI_SOURCES} $<TARGET_OBJECTS:hexapod_core>)
#qt5_use_modules(parameter_gui Core Gui Widgets 3DCore 3DExtras 3DRender 3DInput)
#target_link_libraries(parameter_gui ${SONAME} hebi hebic++ m pthread)

# 2D Parameter setting GUI
add_executable(parameter_gui_2d ${PARAMETER_GUI_2D_SOURCES} $<TARGET_OBJECTS:hexapod_core>)
qt5_use_modules(parameter_gui_2d Core Gui Widgets)
target_link_libraries(parameter_gui_2d ${SONAME} hebi hebic++ m pthread)

add_executable(input_test ${INPUT_TEST_SOURCES})
target_link_libraries( input_test ${SONAME} hebi hebic++ m pthread )

# Add ultra-conservative warnings.
if ( CMAKE_COMPILER_IS_GNUCC )
  set_property( TARGET input_test APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )
endif ( CMAKE_COMPILER_IS_GNUCC )
