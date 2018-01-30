# Hebi C++ Examples

This repo contains examples for the HEBI Robotics C++ API.

## Downloading Dependencies

If cloning the repo directly, you will need to download the C++ API to build the
examples.  Running `python scripts/download_depends.py` will download and
extract the API and associated binaries.

If you have downloaded a tagged release, this step is unnecessary.

## Getting Started

The 'projects' folder contains sample project files for different IDEs/build
systems.

Currently, the only supported build system is CMake (Visual Studio, Makefile,
and XCode support will be added soon).  To get started with CMake, you must
first install CMake.  After doing so, you can run cmake to create the project
files for your desired platform.  For example, on 64 bit Linux, from the root
directory, you can run

```
mkdir build
cd build
cmake -DARCH=x86_64 ../projects/cmake/
```

If CMake is configured to create a Makefile project (the default for Linux), you
can then run `make` to compile the examples.

## Directory Layout

- **hebi** The C++ API and C binaries are stored in this directory (if cloning
the repo directly, note that this directory is created by the `download depends`
script.
- **basic** A set of numbered basic examples that walk through the core concepts
of the API.  This code is the best place to start if you are new (note that to
easily build the examples, use the project files in `projects`.
- **advanced** A set of more involved examples that go through various topics.
- **projects** The directory where IDE and build system files are found.
- **scripts** Helper scripts for downloading the API and releasing the example
package; this directory only exists in the directly cloned repo.

