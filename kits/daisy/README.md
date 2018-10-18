# Example code for Daisy (HEBI Hexapod) #

## Dependencies ##

Ubuntu 18.04 computer with:
* CMake
* Qt 5.9

Other:
* iOS or Android device with HEBI Mobile I/O

To install the dependencies on Ubuntu 18.04:

```
sudo apt install build-essential qt5-default qt3d5-dev cmake

```

## Building ##

Note -- the HEBI C++ API must be in the `hebi-cpp` directory in the root of the
project in order for this project to build correctly!  If this does not exist,
download a github release of the code repository, or build the full examples
CMake (projects/cmake/CMakeLists.txt) in order to download this directory.

This code contains its own CMake project.  It has only been tested in Ubuntu so
far. To compile, from this directory run the following commands:

```
mkdir build;
cd build;
cmake ..
make
```

This will build several different programs and utilities that allow you to
configure and run the hexapod.

## Configuration ##

There are several utility programs included to help set the parameters that
control the robot, as well as test the input device is working as expected.

### Parameter GUI ###

For the parameter GUI, there is a 2D "line art" version and a 3D "spheres and
cylinders" version (`parameter_gui_22` and `parameter_gui`, respectively).

To run these, run `./parameter_gui_2d` or `./parameter_gui` from the build
folder.  These will allow you to load and save the `hex_config.xml` file and
change a couple of the basic values in this file, as well as visualize their
effects.  You can also modify the file directly by hand.

_Note that due to issues installing the Qt53dRender package on Ubuntu 18.04, the
3D rendered parameter GUI has temporarily been disabled and removed from the
CMakeLists scripts.  The 2D GUI has the same functionality and should be used
instead._

Note that the parameter file changed is the one _in the build directory_. You
may want to consider backing up this file if you plan to rebuild. At build time,
CMake copies the file over from the `resources` directory, but whether this
happens each build or just on a brand new build is not specified/guaranteed at
this time.

### Input Test ###

The input test simply prints out data from the connected input device; this is
currently set to be a HEBI "Mobile I/O" app (available for Android and iOS), but
using the interface in the `src/input` directory, one can define a custom input
device as well.

In either case, this program will allow you to verify the basic functionality of
the device before connecting to the full hexapod robot.

## Running ##

To run the control program and drive the robot, run `./hexapod_control`. Note
that there are several command line arguments as well, and a short help message
is displayed if you run `./hexapod_control -h`.

The current command line argument options are:

```
 -d
     Dummy hexapod; no modules are connected to.  Incompatible with "-p".

 -p <list of integer leg indices>
     Creates a partial dummy hexapod; only the legs with the specified indices
     are connected to.

 -v
     Visualize -- show a simple rendering of the robot.

 -h
     Print help and return.
```
