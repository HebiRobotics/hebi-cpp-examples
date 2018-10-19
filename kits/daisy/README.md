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

 -q
     Quiet mode (no dialog messages; waits and tries to continue on failure such as no modules on the network).

 -h
     Print help and return.
```

Note that using `-q` is useful when auto-starting the script on a computer.  The
flag causes the program to not quit when it cannot find the robot or joystick,
(or the robot is M-Stopped), but continually try again. It uses a simple set of
LED codes:

When it can find the robot, but one of the legs is out of a safe range (+/- 90
degrees from straight out), it sets the LEDs on that leg red.

When if can find the robot, and all of the legs are within range, but it cannot
find the joystick (mobile IO app with family "HEBI" and name "Mobile IO", it
sets the LEDs on all modules to blue.

## Driving ##

After starting the program, the robot will take a few seconds to move the legs
into a starting stance and then raise its body.

When driving the robot, there are two basic modes - "stance" and "step".
Pressing "B7" on the Mobile IO app will toggle between these modes.

### Step ###

By default, the robot is in the "step" mode; it will take a step to keep balanced
as its body/center of mass is moved (either when manually moved or when driven
with the app). 

The left joystick can be used to rotate in place, and the right joystick can be
used to move forward/back or sidestep.

### Stance ###

In stance mode, the legs stay on the ground, and the body can move relative to
this stance position.  The left joystick is used for tilting and rotating the
body, and the right joystick is used for shifting the body forward/back and
left/right.

### Other Controls ###

In both modes, the slider A3 can be used to move the body up and down. The
slider controls the velocity of the motion; the middle 50% is a "dead zone" that
does not command a height change.

As noted above, "B7" switches between Step and Stance modes.

The button "B8" will quit the app.

## Automatic startup instructions for Linux

This process was tested using Ubuntu 16.04.

The folder also contains scripts designed to automatically launch the demo.
`daisyStart.sh` is used to launch the demo with the correct path. The
`install.sh` script will create a symbolic link to `daisyStart.sh` so that the
user can simply type `daisyStart` into a terminal window and program will run.

To install:
* Open a command prompt and navigate to the top level `daisy` kit directory
* Make sure that the `install.sh` script has execute permissions: `chmod +x install.sh`
* Run `install.sh`: `./install.sh`

If you would like to have the script start on boot, add the `daisyStart`
command under `Startup Applications` if using Ubuntu or use another standard
method for automatically launching the program.

Note that if you move the location of `daisyStart.sh` you will have to delete
the symbolic link from `/usr/bin` and run `install.sh` again.
