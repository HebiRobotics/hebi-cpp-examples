cmake_minimum_required(VERSION 3.10)
project(mav_trajectory_generation)

set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig)
find_package(Eigen3 REQUIRED)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)

# Add eigen_checks directory to the include path
set(EIGEN_CHECKS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../eigen_checks/include")

# Find NLopt package
find_package(NLopt)
if(NOT NLOPT_FOUND)
  # Try pkg-config if the regular find_package doesn't work
  pkg_check_modules(NLOPT nlopt)
endif()

# Set include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}  # Add Eigen3 include directories
  ${YamlCpp_INCLUDE_DIRS}
  ${EIGEN_CHECKS_INCLUDE_DIR}  # Add eigen_checks include directory
  ${NLOPT_INCLUDE_DIRS}  # Add NLopt include directories
)

#############
# LIBRARIES #
#############
set(LIB_SOURCES
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/vertex.cpp
  src/rpoly/rpoly_ak1.cpp
)

add_library(${PROJECT_NAME} ${LIB_SOURCES})
target_link_libraries(${PROJECT_NAME} ${YamlCpp_LIBRARIES})

##########
# INSTALL #
##########
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)
