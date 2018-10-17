= Example code for Daisy (HEBI Hexapod) =

== Dependencies ==

CMake
Qt
iOS or Android device with HEBI Mobile I/O

*TODO: get version for Qt and CMake, and any other dependencies*

== Building ==

Note -- the HEBI C++ API must be in the `hebi-cpp` directory in the root of the
project in order for this project to build correctly!  If this does not exist,
download a github release of the code repository, or build the full examples
CMake (projects/cmake/CMakeLists.txt) in order to download this directory.

This code contains its own CMake project.  It has only been tested in Linux so
far. To compile, from this directory run the following commands:

```
mkdir build;
cd build;
cmake .. -DQT58BASE=/path/to/Qt/5.8 -DQT59BASE=/path/to/Qt/5.9
make
```

This will build several different programs and utilities that allow you to
configure and run the hexapod.

== Configuration ==

*TODO: write instructions*

== Running ==

*TODO: write instructions, including for each program and any command line args*
