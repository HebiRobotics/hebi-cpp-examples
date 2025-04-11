# Used to download the C++ API - this should not be used directly.
cmake_minimum_required(VERSION 3.5)

set(HEBI_CPP_VERSION "3.12.2")
set(HEBI_CPP_FILE_NAME "hebi-cpp-${HEBI_CPP_VERSION}.tar.gz")
set(HEBI_CPP_LIB_SHA256 "308ea66b0671e70dd1afea30e39c54e1c11cb2c92ae3981cf7c272f3d5971ece")
set(HEBI_CPP_URL "https://files.hebi.us/download/cpp/${HEBI_CPP_FILE_NAME}")

# If the CMakeLists.txt is not found, then redownload the C++ API 
if(NOT EXISTS ${HEBI_DIR}/CMakeLists.txt)

message(STATUS "Downloading C++ API version ${HEBI_CPP_VERSION} from ${HEBI_CPP_URL}")
get_filename_component(HEBI_CPP_FILE_FULL_PATH "${CMAKE_BINARY_DIR}/${HEBI_CPP_FILE_NAME}" REALPATH)
file(DOWNLOAD "${HEBI_CPP_URL}" "${HEBI_CPP_FILE_FULL_PATH}"
  EXPECTED_HASH SHA256=${HEBI_CPP_LIB_SHA256}
  SHOW_PROGRESS)

# Try and find a python install
find_program(PYTHON_EXECUTABLE python)

if(EXISTS "${PYTHON_EXECUTABLE}")
  execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
  "import tarfile; tar = tarfile.open(r'${HEBI_CPP_FILE_FULL_PATH}'); tar.extractall(path=r'${ROOT_DIR}'); tar.close()")
# Fall back on `tar` for MacOS and Linux
elseif(NOT EXISTS "${PYTHON_EXECUTABLE}" AND NOT WIN32)
  execute_process(COMMAND "tar" "-xvf" "${HEBI_CPP_FILE_FULL_PATH}"
    WORKING_DIRECTORY ${ROOT_DIR})
elseif(WIN32)
  message(FATAL_ERROR "Cannot extract ${HEBI_CPP_FILE_FULL_PATH} because python was not found in the executable path. Extract this file manually and re-run CMake.")
endif()

endif()
