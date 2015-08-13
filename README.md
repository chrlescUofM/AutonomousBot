# AutonomousBot
My work with the MAEBot platform building an autonomous robot. Implements scan matching SLAM, DStarLite path planning, and PID control.  My primary task was in the implementation of the DStarLite algorithm.

## Repository Structure
Note: Some files are omitted for brevity.
* AutonomousBot -- Top level directory.
  * [calibration.config](/calibration.config) -- Contains the intrinsics and extrinsics camera parameters.
  * Makefile -- Use this to compile the code.
  * README.md -- This file.
  * [results.pdf](/results.pdf) -- An overview of the methods and results achieved.
  * [setenv.sh](/setenv.sh) -- A script to set up adjustable enviornment variables.
  * [bin/](/bin/) -- Contains the final executable files.
  * [config/](/config/) -- Necessary configuration files to run applications remotely on MAEBot.
  * [lcmtypes/](/lcmtypes/) -- Defines the message types that will be sent using LCM to communicate with the hardware.
  * [lib/](/lib/) -- Contains necessary archives.
  * [src/](/src/) -- Contains the source code.
  	* [bot/](/src/bot/) -- Contains the autonomous bot source code.
  	  * [botlab.c](/src/bot/botlab.c) -- Main program and control loop for execution.
  	  * [dStarLite.c](/src/bot/dStarLite.c) -- An implementation of DStarLite path planning.
  	  * [pidControl.c](/src/bot/pidControl.c) -- Implements tuned PID control for the MAEBot.
  	  * [priorityQueue.c](/src/bot/priorityQueue.c) -- Implements a C interface for a priority Queue structure.
  	  * [scan_matching.c](/src/bot/scan_matching.c) -- Implements scan matching algorithm to align lidar scans.
  	  * [slam.c](/src/bot/slam.c) -- Implements Simultaneous Localization and Mapping
  	  * [smooth.c](/src/bot/smooth.c) -- Implements an optional path smoothing algorithm.
    * [common/](/src/common/) -- Contains C implementations of usefull C++ constructs.
    * [imagesource/](/src/imagesource/) -- Contains code for transmitting images from the MAEBot.
    * [maebot/](/src/maebot/) -- Contains driver code for the MAEBot platform.
    * [math/](/src/math/) -- Contains C implementations of usefull robotics functions.
    * [pixy/](/src/pixy/) -- Contains pixy camera driver.
    * [vx/](/src/vx/) -- Contains visualization code.


## Necessary Packages
To begin with you should install the following packages, some of these are not necessary but are useful to have:

```
sudo apt-get -y install emacs vim-gtk cvs subversion git-core gitk units octave imagemagick ssh smartmontools nmap netcat curl wireshark traceroute libav-tools mplayer vlc sysstat valgrind autopoint xchat mesa-utils pkg-config curl

sudo apt-get -y install autoconf automake autotools-dev libglib2.0-dev manpages-dev manpages-posix-dev libgl1-mesa-dev gtk-doc-tools libgtk2.0-dev python-dev libusb-dev libusb-1.0-0-dev libfuse-dev libi2c-dev libdc1394-22-dev libdc1394-utils libgsl0-dev gsl-doc-info gsl-doc-pdf realpath

sudo apt-get -y install ant openjdk-6-jdk
```
You will also need to install LCM.  You can download version lcm-1.1.1 or higher from http://code.google.com/p/lcm, or alternatively, you can pull the latest source using git.  You will then to build and install from source following LCM's install directions.  Be sure to run ldconfig after having built LCM so that the linker can find it.
```
git clone https://code.google.com/p/lcm/
cd lcm && ./bootstrap.sh && ./configure && make && sudo make install
sudo ldconfig
```
As a one time thing, you need to build vx's fonts (you will need a working internet connection for this so that the Makefile can pull a fonts tarball from the web):
```
cd /src/vx
make fonts
```
You are now all set to build the source using `make` or `make clean`

## Usage Instructions
Running make inside the src/botlab directory compiles all the programs and puts the binaries inside the bin/ directory.

NOTE:

* To simplify the initialization process (to setup the procman deputies, sherriff, lcm tunnel etc) we've created two 
scripts. The setup_bot.sh script must be run on the maebot. This will set the required environmental variables, create 
the deputies and establish an lcm tunnel. The setup_lap.sh must be run on the lab laptop. This will setup all the 
necessary prerequisites and open up the procman sherriff.

* The odometry and lidar drivers must be run on the maebot prior to running the applications described below.

###botlab_odometry
botlab_odometry is obtained from compiling the odometry.c program. We've programmed two different odometry models. 
The default model uses the data obtained from the wheel encoders to estimate the x,y position and the angle relative 
to an inertial frame. If the binary is executed with the "--use-gyro" flag, the second model that uses the wheel 
encoders and the gyro to determine the pose of the robot. The alpha and beta parameters for describing the covaraince 
in the longitudinal and lateral slip is specified as defines inside the odometry.c file. If the application is started 
with the "--use-gyro", the robot must remain stationary for the first 5 seconds. This enables us to obtain an estimate 
of the gyro bias and hence eliminate the gyro bias. 

###botlab_scanmatcher
This app should be run after the gyro has been calibrated in the previous app. This program estimates poses based on 
scan matching and publishes these pose estimates over an LCM channel.

###botlab_app
This is the main engine that initiates mapping, planning and control. Running the VX remote viewer provides GUI 
for visualization. SHIFT+e turns on the PID controller. SHIFT+r turns off the PID controller.