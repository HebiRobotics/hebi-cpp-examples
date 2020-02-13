# Toolchain file for targeting i686 on Linux. This is primarily designed
# to be used on an x86_64 based 'multilib' Linux machine.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR "i686")

set(LIBHEBI_TARGET_ARCHITECTURE "i686" CACHE STRING "Target architecture. Do not modify." FORCE)

# Need to force the compiler to compile 32 bit code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
