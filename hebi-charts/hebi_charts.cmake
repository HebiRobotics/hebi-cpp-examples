# Used to download the binaries - this should not be used directly.
cmake_minimum_required(VERSION 3.12)

# ======== Determine target platform ========
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(HEBI_CHARTS_OS "linux")
  set(HEBI_CHARTS_LIB_NAME "libhebi_charts.so")
  set(HEBI_CHARTS_LINK_NAME "libhebi_charts.so")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(HEBI_CHARTS_OS "osx")
  set(HEBI_CHARTS_LIB_NAME "libhebi_charts.dylib")
  set(HEBI_CHARTS_LINK_NAME "libhebi_charts.dylib")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  set(HEBI_CHARTS_OS "win")
  set(HEBI_CHARTS_LIB_NAME "hebi_charts.dll")
  set(HEBI_CHARTS_LINK_NAME "hebi_charts.lib")
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
  set(HEBI_CHARTS_ARCH "amd64") # For 64-bit
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(HEBI_CHARTS_ARCH "arm64") # For ARM64
endif()

set(HEBI_CHARTS_PLATFORM "${HEBI_CHARTS_OS}_${HEBI_CHARTS_ARCH}")

# ======== Variables to be used by including projects ========
set(HEBI_CHARTS_VERSION "0.5.0")
set(HEBI_CHARTS_BUILD_NUMBER "67")
set(HEBI_CHARTS_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include/")
set(HEBI_CHARTS_LIB_FILE "${CMAKE_CURRENT_LIST_DIR}/lib/${HEBI_CHARTS_PLATFORM}/${HEBI_CHARTS_LIB_NAME}")
set(HEBI_CHARTS_LINK_FILE "${CMAKE_CURRENT_LIST_DIR}/lib/${HEBI_CHARTS_PLATFORM}/${HEBI_CHARTS_LINK_NAME}")

# ======== Download binaries on-demand ========
option(DOWNLOAD_HEBI_CHARTS "Download binaries for the local system if none are available" ON)
if(DOWNLOAD_HEBI_CHARTS AND NOT EXISTS "${HEBI_CHARTS_LIB_FILE}")

  # Build URL
  set(HEBI_CHARTS_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/")
  set(HEBI_CHARTS_DL_NAME "hebi_charts-${HEBI_CHARTS_VERSION}-${HEBI_CHARTS_PLATFORM}.zip")
  set(HEBI_CHARTS_DL_URL "https://files.hebi.us/download/hebi_charts/snapshot/${HEBI_CHARTS_BUILD_NUMBER}/${HEBI_CHARTS_DL_NAME}")

  # Download full release
  message(STATUS "Downloading hebi_charts binaries for ${HEBI_CHARTS_PLATFORM} version ${HEBI_CHARTS_VERSION} from ${HEBI_CHARTS_DL_URL}")
  get_filename_component(HEBI_CHARTS_DL_PATH "${CMAKE_BINARY_DIR}/${HEBI_CHARTS_DL_NAME}" REALPATH)
  file(DOWNLOAD "${HEBI_CHARTS_DL_URL}" "${HEBI_CHARTS_DL_PATH}" SHOW_PROGRESS)

  # Extract to a temporary dir and copy only the binaries
  message(STATUS "Extracting hebi_charts binaries from ${HEBI_CHARTS_DL_PATH} to ${HEBI_CHARTS_LIB_DIR}")
  set(TEMP_DIR "${CMAKE_BINARY_DIR}/temp.XXXXXX")

  file(MAKE_DIRECTORY "${TEMP_DIR}")
  file(ARCHIVE_EXTRACT
       INPUT "${HEBI_CHARTS_DL_PATH}"
       DESTINATION "${TEMP_DIR}")

  file(COPY "${TEMP_DIR}/hebi_charts/lib/${HEBI_CHARTS_PLATFORM}" DESTINATION "${HEBI_CHARTS_LIB_DIR}")
  file(REMOVE_RECURSE "${TEMP_DIR}")
  file(REMOVE "${HEBI_CHARTS_DL_PATH}")

endif()
