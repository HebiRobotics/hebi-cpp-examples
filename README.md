# Hebi C++ Examples

This repo contains examples for the HEBI Robotics C++ API.

Note that many of these programs contains hardcoded module family/name values,
and these values should be modified to match the modules available on your
network.

## Downloading Dependencies

The C++ API should reside in a folder called `hebi-cpp` in the root directory of this repository. This must be downloaded prior to building, and there are two ways to do this:

1. Using the CMake project
   - Follow the instructions under [CMake](#cmake) and the dependencies will be automatically downloaded.
   - The API's version is specified in `projects/cmake/DownloadHebiCpp.cmake` through:
    ```
    set(HEBI_CPP_VERSION "x.x.x")
    ```
    along with its corresponding hash:
    ```
    set(HEBI_CPP_LIB_SHA256 "longstringofcharacters")
    ```
    - To change this, first edit the version field, and run the `cmake` command in the build directory. This should give you an error containing the expected hash. Swap this in and run the `cmake` command again.

2. Downloading Manually
   - Prior to building, you can download your preferred version from [the docs](http://docs.hebi.us/downloads_changelogs.html#software).
   - Extract the `hebi-cpp` folder from the `hebi-cpp-x.x.x` folder into the root directory of this repository.

For examples which use plotting, python is needed. To install it, execute "sudo apt-get install python3-matplotlib python3-numpy python3.6-dev" in terminal. (Plotting is enabled using a header file matplotlib.h which is a modified version of the one found at https://github.com/lava/matplotlib-cpp)

Use the same version of the C++ API that is defined at the top of projects/cmake/DownloadHebiCpp.cmake, as this will ensure all the code is compatible.

## Getting Started

The 'projects' folder contains sample project files for different IDEs/build
systems.

Currently, the recommended build systems is CMake, although it is possible to add the source files and library dependencies in Visual Studio, XCode, Makefiles, etc.

### CMake

To get started with CMake, you must
first install CMake.  After doing so, you can run cmake to create the project
files for your desired platform.  You can run `cmake --help` to get platform-specific
options, but here are a couple basic examples:

**64-bit (aarch64 or x86_64) Linux or MacOS; generates a makefile project**
 
From the root directory of the checkout, run:

```
mkdir build
cd build
cmake ../projects/cmake/
```

Note -- for Linux aarch64 arm builds, change the final line to:
```
cmake -DLIBHEBI_TARGET_ARCHITECTURE=aarch64 ../projects/cmake/
```

For MacOS arm64 (M1) builds, change the final line to:
```
cmake -DLIBHEBI_TARGET_ARCHITECTURE=arm64 ../projects/cmake/
```

If CMake is configured to create a Makefile project (the default for Linux/Mac command
line), you can then run

```make```

to compile the examples.

**64-bit Windows; Visual Studio**

From the root directory of the checkout, run the following in a command prompt:

```
mkdir build
cd build
cmake -G"Visual Studio 16 2019" -Ax64 ../projects/cmake/
```

Then open the `build/hebi_cpp_examples.sln` solution file, set the startup project
to your desired example, and hit the green "run" button at the top of the IDE to
compile and run it.

For other versions of Visual Studio, `cmake --help` will provide the exact syntax.
For example, for 64-bit builds on Visual Studio 2017, change the final line to:

```
cmake -G"Visual Studio 15 2017 Win64" ../projects/cmake/
```

Note that if you have not installed the debug symbols when you installed python, then you will need to compile in "Release" mode for examples which link against the python runtime.

## Directory Layout

- **hebi-cpp** The C++ API and C binaries are stored in this directory (if cloning
the repo directly, note that this directory is created by CMake automatically, or
you can unzip the C++ API manually into this folder, using the same version as listed
at the top of projects/cmake/DownloadHebiCpp.cmake)
- **basic** A set of numbered basic examples that walk through the core concepts
of the API.  This code is the best place to start if you are new (note that to
easily build the examples, use the project files in `projects`.
- **advanced** A set of more involved examples that go through various topics.
- **kits** Full-system example code for the various kits that HEBI produces.
- **projects** The directory where IDE and build system files are found.
- **util** Various utility functions that can help with control and full system examples.  For example, joystick input code and gravity compensation code are contained here.

