# Hebi C++ Examples

This repo contains examples for the HEBI Robotics C++ API.

Note that many of these programs contains hardcoded module family/name values,
and these values should be modified to match the modules available on your
network.

## Downloading Dependencies

If cloning the repo directly, you will need to download the C++ API from http://docs.hebi.us/downloads_changelogs.html#software to build the examples and place it in a folder called `hebi-cpp`. If you use the CMake project, it will automatically be downloaded, assuming you have a working internet connection.

For examples which use plotting, matplotlibcpp is needed. This can be downloaded and installed by following instructions here: https://github.com/lava/matplotlib-cpp

Use the same version of the C++ API that is defined at the top of projects/cmake/DownloadHebiCpp.cmake, as this will ensure all the code is compatible.

## Getting Started

The 'projects' folder contains sample project files for different IDEs/build
systems.

Currently, the only supported build systems are CMake and Visual Studio (Makefile
and XCode support will be added soon).

### Visual Studio

Open the `projects/visual_studio/basic_examples.sln` file in Visual Studio.
VS 2017 has been tested, but earlier versions are expected to work as well.
(Note that a VS project for the advanced examples is coming soon!)

To run an example:

- Right click on the desired example project in the solution explorer, and
click "Set as StartUp Project",
- Select the desired configuration (Debug/Release and x64/Win32) from the
top toolbar.
- Click the run button (green triangle, beside the configuration selection
buttons) to run the program.

Note that as these are console applications with no pause at the end, you
may wish to add a breakpoint before the program returns to have a chance
to inspect the console output.

### CMake

To get started with CMake, you must
first install CMake.  After doing so, you can run cmake to create the project
files for your desired platform.  For example, on 64 bit Linux, from the root
directory, you can run

```
mkdir build
cd build
cmake -DARCH=x86_64 ../projects/cmake/
```

If CMake is configured to create a Makefile project (the default for Linux), you
can then run

```make```

to compile the examples.

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

