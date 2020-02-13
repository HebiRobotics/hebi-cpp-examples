cmake_minimum_required(VERSION 3.0)

project(libhebi VERSION 2.0.0)

add_library(hebi SHARED IMPORTED GLOBAL)

if (UNIX AND NOT APPLE)

  #[[
    It is assumed that you are targeting x86_64. If you would like
    to target another architecture, use one of the toolchain files
    included with this library.
  #]]

  set(LIBHEBI_TARGET_ARCHITECTURE "x86_64" CACHE STRING
    "Target architecture. [i686, x86_64, armhf, aarch64]")
  set(LIBHEBI_LOCATION "lib/linux_${LIBHEBI_TARGET_ARCHITECTURE}")
elseif (APPLE)
  set(LIBHEBI_LOCATION "lib/osx_amd64")
elseif (WIN32)

  # Handle visual studio architecture stuff
  if ("" STREQUAL "${CMAKE_VS_PLATFORM_NAME}")
    message(WARNING
      "It is recommended that you use Visual Studio for Windows development. \
      You will have to manually specify the target architecture for your \
      currently specified generator '${CMAKE_GENERATOR}'.")
  elseif("Win32" STREQUAL "${CMAKE_VS_PLATFORM_NAME}")
    set(HEBI_VS_ARCH "x86")
  elseif("x64" STREQUAL "${CMAKE_VS_PLATFORM_NAME}")
    set(HEBI_VS_ARCH "x64")
  elseif("ARM" STREQUAL "${CMAKE_VS_PLATFORM_NAME}")
    message(FATAL_ERROR "Currently unsupported Windows architecture")
  elseif("ARM64" STREQUAL "${CMAKE_VS_PLATFORM_NAME}")
    message(FATAL_ERROR "Currently unsupported Windows architecture")
  else()
    message(FATAL_ERROR "Unkown value for CMAKE_VS_PLATFORM_NAME (${CMAKE_VS_PLATFORM_NAME}")
  endif()

  set(LIBHEBI_TARGET_ARCHITECTURE "${HEBI_VS_ARCH}"
    CACHE STRING "Target architecture. [x86, x64, ARM, ARM64]")
  set(LIBHEBI_LOCATION "lib/win_${LIBHEBI_TARGET_ARCHITECTURE}")

  set_property(TARGET hebi PROPERTY IMPORTED_IMPLIB
    "${CMAKE_CURRENT_SOURCE_DIR}/${LIBHEBI_LOCATION}/${CMAKE_IMPORT_LIBRARY_PREFIX}hebi${CMAKE_IMPORT_LIBRARY_SUFFIX}")
endif()

set_property(TARGET hebi APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
  "${CMAKE_CURRENT_SOURCE_DIR}/include" )

set_property(TARGET hebi PROPERTY IMPORTED_LOCATION
  "${CMAKE_CURRENT_SOURCE_DIR}/${LIBHEBI_LOCATION}/${CMAKE_SHARED_LIBRARY_PREFIX}hebi${CMAKE_SHARED_LIBRARY_SUFFIX}")

set_property(TARGET hebi PROPERTY INTERFACE_LINK_LIBRARIES
  $<$<BOOL:${UNIX}>:m pthread>
  $<$<BOOL:${WIN32}>:kernel32>)
