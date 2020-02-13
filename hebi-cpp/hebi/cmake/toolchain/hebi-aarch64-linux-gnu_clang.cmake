# Toolchain file for targeting aarch64 on Linux using the HEBI XDK Clang toolchain

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(LIBHEBI_TARGET_ARCHITECTURE "aarch64" CACHE STRING "Target architecture. Do not modify." FORCE)
set(target_triple "aarch64-linux-gnu")

# Hacky way to get the path to the cmake file in the _actual_ source directory.
# We can't access the source directory in this file, since this file is run
# before the `CMakeLists.txt` in the source directory.
get_filename_component(currdir ${CMAKE_CURRENT_LIST_FILE} ABSOLUTE)
get_filename_component(currdir ${currdir} DIRECTORY)
include(${currdir}/util/hebi-xdk.cmake)

# If you want to specify an absolute path or variant name of your clang executable (e.g., clang-6.0)
# Then set them below
#set(clang_exec /usr/local/bin/clang)
#set(CMAKE_C_COMPILER          ${clang_exec})
#set(CMAKE_CXX_COMPILER        ${clang_exec}++)
set(CMAKE_C_COMPILER_TARGET   ${target_triple})
set(CMAKE_CXX_COMPILER_TARGET ${target_triple})

include(${currdir}/util/xdk-cross-bin-locations.cmake)
set(CMAKE_SYSROOT_COMPILE ${hebi_xdk_sysroot})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LIBHEBI_CROSS_GCC_TOOLCHAIN_ARG}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${LIBHEBI_CROSS_GCC_TOOLCHAIN_ARG}")