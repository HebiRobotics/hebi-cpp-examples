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
  ${GLOG_INCLUDE_DIRS}
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
target_link_libraries(${PROJECT_NAME} ${YamlCpp_LIBRARIES} ${GLOG_LIBRARIES} glog)

############
# BINARIES #
############
add_executable(polynomial_timing_evaluation
  src/polynomial_timing_evaluation.cpp
)
target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME} ${GLOG_LIBRARIES} glog)

#########
# TESTS #
#########
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
  enable_testing()
  
  # Find GTest
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})
  
  add_executable(test_polynomial test/test_polynomial.cpp)
  target_link_libraries(test_polynomial ${PROJECT_NAME} ${GTEST_LIBRARIES} ${GLOG_LIBRARIES} glog pthread)
  add_test(NAME test_polynomial COMMAND test_polynomial)
  
  add_executable(test_polynomial_optimization test/test_polynomial_optimization.cpp)
  target_link_libraries(test_polynomial_optimization ${PROJECT_NAME} ${GTEST_LIBRARIES} ${GLOG_LIBRARIES} glog pthread nlopt)
  add_test(NAME test_polynomial_optimization COMMAND test_polynomial_optimization)
endif()

##########
# INSTALL #
##########
install(TARGETS ${PROJECT_NAME} polynomial_timing_evaluation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)
