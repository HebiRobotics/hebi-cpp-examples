#[[
  Internal CMake script used for HEBI XDK toolchains. Do not use directly.
#]]

# Uses gcc-8.2.0 toolchain by default
# This is currently the only gcc version available in the XDK.
set(hebi_xdk_gcc_version "8.2.0")

# Path to gcc toolchain to use.
# This is currently the only gcc version available in the XDK.
set(hebi_xdk_toolchain_path "/opt/hebi/toolchains/${target_triple}-${hebi_xdk_gcc_version}")

# Sysroot for target platform
set(hebi_xdk_sysroot "/opt/hebi/arch/${target_triple}")

set(LIBHEBI_CROSS_GCC_TOOLCHAIN_ARG "-B${hebi_xdk_sysroot}/bin" CACHE STRING "" FORCE)

# Clang also requires a GCC toolchain, so setting this when using clang
# is necessary, since we want to use all of the HEBI cross compiler binaries.
set(CMAKE_C_COMPILER_EXTERNAL_TOOLCHAIN ${hebi_xdk_sysroot})
set(CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN ${hebi_xdk_sysroot})

# TODO: See if this is actually necessary
#set(CMAKE_LINKER ${hebi_xdk_toolchain_path}/bin/${target_triple}-ld
#  CACHE FILEPATH "" FORCE)

# Sets CMake in correct state for cross compiling
#set(CMAKE_SYSROOT        ${hebi_xdk_sysroot})
set(CMAKE_FIND_ROOT_PATH ${hebi_xdk_sysroot})
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
