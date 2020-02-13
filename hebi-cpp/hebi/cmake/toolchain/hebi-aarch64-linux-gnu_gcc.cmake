# Toolchain file for targeting aarch64 on Linux using the HEBI XDK GCC toolchain

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

set(CMAKE_C_COMPILER   ${hebi_xdk_toolchain_path}/bin/${target_triple}-gcc)
set(CMAKE_CXX_COMPILER ${hebi_xdk_toolchain_path}/bin/${target_triple}-g++)
