rpg_dvs_ros
===========

# Disclaimer and License

The RPG ROS DVS package has been tested under ROS Indigo and Ubuntu 14.04.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a **GNU General Public License (GPL)**.


# Package Overview

The ROS DVS package provides a C++ driver for the Dynamic Vision Sensor (DVS).
It also provides a calibration tool for both intrinsic and stereo calibration.
To find out more about the DVS, visit the website of the [Institute of Neuroinformatics](http://siliconretina.ini.uzh.ch/wiki/index.php).

Authors: Elias Mueggler, Basil Huber, Luca Longinotti, Tobi Delbruck

## Publications

If you use this work in an academic context, please cite the following publications:

* E. Mueggler, B. Huber, D. Scaramuzza: **Event-based, 6-DOF Pose Tracking for High-Speed Maneuvers**. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Chicago, 2014. ([PDF](http://rpg.ifi.uzh.ch/docs/IROS14_Mueggler.pdf))
* A. Censi, J. Strubel, C. Brandli, T. Delbruck, D. Scaramuzza: **Low-latency localization by Active LED Markers tracking using a Dynamic Vision Sensor**. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, 2013. ([PDF](http://rpg.ifi.uzh.ch/docs/IROS13_Censi.pdf))
* P. Lichtsteiner, C. Posch, T. Delbruck: **A 128×128 120dB 15us Latency Asynchronous Temporal Contrast Vision Sensor**. IEEE Journal of Solid State Circuits, Feb. 2008, 43(2), 566-576. ([PDF](https://www.ini.uzh.ch/~tobi/wiki/lib/exe/fetch.php?media=lichtsteiner_dvs_jssc08.pdf))


# Driver Installation

Install **catkin_simple** from https://github.com/catkin/catkin_simple.git

Install **libcaer** from https://svn.code.sf.net/p/jaer/code/libcaer/trunk/ ([Instructions](http://inilabs.com/support/software/libcaer/)) -- note that you will need gcc 4.9 or higher

Make sure, libusb is installed on your system:  
1. `$ sudo apt-get install libusb-1.0-0-dev`

Only a udev rule is needed to run the DVS driver. An install script is provided in the package dvs_driver.  
2. `$ roscd libcaer_catkin`  
3. `$ ./install.sh` (needs root privileges)

You can test the installation by running a provided launch file. It starts the driver (DVS or DAVIS), the renderer, an image viewer, and the dynamic reconfigure GUI.   
4. `$ roslaunch dvs_renderer dvs_mono.launch`  
5. `$ roslaunch dvs_renderer davis_mono.launch`  

## Very detailed example of installation (tested on Ubuntu 14.04.01 LTS)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update 
sudo apt-get install ros-jade-desktop
sudo apt-get install ros-jade-camera-info-manager
sudo rosdep init
rosdep update
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc


#create project folder
mkdir -p catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace 
cd ..
catkin_make
#working
#install source
cd src/
git clone git@github.com:lalten/rpg_dvs_ros.git
git clone git@github.com:catkin/catkin_simple.git
#got some errors for missing libraries
#see https://github.com/lalten/rpg_dvs_ros
sudo apt-get install libusb-1.0-0-dev

#install libcaer for dvs128
cd /tmp/
svn checkout https://svn.code.sf.net/p/jaer/code/libcaer/trunk/
cd cmake -DCMAKE_INSTALL_PREFIX=/usr
make
sudo make install
#got compile errors, install gcc 4.9
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install g++-4.9
sudo rm /usr/bin/gcc
sudo ln -s /usr/bin/gcc-4.9 /usr/bin/gcc
#now try install again
make
sudo make install

#now go to project folder
#make project
cd catkin_ws/
catkin_make

#run project
source devel/setup.bash
roslaunch dvs_calibration intrinsic_edvs.launch

#furthermore, for some tools of the packae this are neccessary
#for dvs_render
sudo apt-get install ros-jade-image-view

#for rosrun image_view 
sudo aptitude install ros-jade-stereo-image-proc 
```

## Build Issues after Update / git pull

Sometimes, there seems that catkin_make does not refresh all files.
E.g. if you get an error message of a missing header file, try the following:
- Edit dvs_msgs/CMakeLists.txt (e.g. add just a new line and store it)
- run catkin_make clean
- run catkin_make

Now the new header files get also built.

# DVS Calibration
The calibration of a DVS is a two-stage procedure. First, the focus must be adjusted. Then, the intrinsic camera parameters are estimated.   

## Focus Adjustment
Adjust the focus of the DVS. One way of achieving this is using a special pattern, e.g. the [Back Focus Pattern](https://github.com/uzh-rpg/rpg_dvs_ros/blob/master/dvs_calibration/pdf/backfocus.pdf).

## Instrisic Parameters
To run the intrinsic camera calibration, we use a 5x5 LED board that is blinking at 500Hz.
The calibration procedure is then started using  
`$ roslaunch dvs_calibration dvs_intrinsic.launch`  
or, for the DAVIS,  
`$ roslaunch dvs_calibration davis_intrinsic.launch`  
You will see an RQT interface with all necessary information.
Top left is the calibration GUI, which displays the amount of detected patterns.
**Currently, pattern detection does not seem to work indoors. Try close to a window.**
Collect at least 30 samples before starting the calibration.
**This can take up to a few minutes** and freezes your RQT GUI.
Once done, the calibration parameters are shown and can be saved.
The camera parameters will be stored in `~/.ros/camera_info`.
When you plug that DVS again, this calibration file will be loaded and published as `/dvs/camera_info`. 

The image viewers below show the following:

1. Accumulated DVS renderings: you should see the blinking LEDs and the gradients in the scene
2. Detected blinking: black regions mean more detections. Once the pattern is detected, the counter in the calibration GUI should increment. The detected pattern is also visualized for a short moment.
3. Rectified DVS rendering: once the calibration is done, you can see how well it turned out. Check if straight lines are still straight, especially in the border of the image.


# Stereo DVS Calibration

## Setup
Connect the two DVS from OUT (master) to IN (slave). 
GND must not be connected if both DVS are connected over USB to the same computer, to avoid ground loops.
Time synchronization is performed automatically in the driver software.
Since each DVS has a separate driver, the ROS messages might arrive at different times. 
Hover, the timestamps within the messages are synchronized.

## Calibration
1. Calibrate each DVS independently
2. Use `$ roslaunch dvs_calibration dvs_stereo.launch`  
3. Use the same checkerboard with blinking LEDs and make sure it is visible in both cameras. Collect at least 30 samples.
4. Start the calibration and check the reprojection error. Then save it (this will extend your intrinsic camera info files with the stereo information).


# Calibration Details and Parameters
The calibration requires a board with a regular grid of blinking LEDs.
In our case we have a 5x5 grid with a 0.05m distance between the LEDs. 
One of the rows can be turned off (to make a 5x4 grid) to avoid confusion in the stereo case.
The following parameters can be tuned using ROS parameters:
* `dots_w`, `dots_h` (default: 5) is the number of rows and columns in the grid of LEDs
* `dot_distance` (default: 0.05) is the distance in **meters** between the LEDs

If you have your own LED board with different LEDs or blinking frequencies, you might want to tweak these parameters as well:
* `blinking_time_us` (default: 1000) is the blinking time in **micro**-seconds
* `blinking_time_tolerance_us` (default: 500) is the tolerance in **micro**-seconds to still count the transition
* `enough_transitions_threshold` (default: 200) is the minimum number of transitions before searching the LEDs
* `minimum_transitions_threshold` (default: 10) is the minimum number of transitions required to be considered in the LED search
* `minimum_led_mass` (default: 50) is the minimum "mass" of an LED blob, i.e., the sum of transitions in this blop
* `pattern_search_timeout` (default: 2.0) is the timeout in **seconds** when the transition map is reset (it is also reset when the LED grid was found)

# Tools

## Store detected patterns

In order to analyze, which points where detected, one can record the result from two topics:
- /dvs_calibration/detected_points_left_or_single
- /dvs_calibration/detected_points_left_or_single_pattern


Here a short example, how to e.g. record and view the data:
```
#first source resources
source devel/setup.bash

#run the calibration interface
roslaunch dvs_calibration intrinsic_edvs.launch

#now, on e.g. other terminal
#this will display only the transition maps with detected points
rosrun image_view image_view image:=/dvs_calibration/detected_points_left_or_single_pattern &

#this will show the detected points messages
rostopic echo /dvs_calibration/detected_points_left_or_single &

#this will record both of them in a file
rosbag record /dvs_calibration/detected_points_left_or_single_pattern /dvs_calibration/detected_points_left_or_single
```

For playback of the recorded data, run:
```
roscore &

#this will show the detected points messages
rostopic echo /dvs_calibration/detected_points_left_or_single &

#play in an endless loop the file
rosbag play -l 2015-12-14-15-17-38.bag 

#or step through the file: press p in the command window or
#space to pause playback
#rosbag play --pause 2015-12-14-15-17-38.bag 
```

# Troubleshooting
## New dvs_msgs format
If you recorded rosbags with a previous version of this package, they must be migrated. 
The format for the timestamps changed from uint64 to rostime.
To convert an "old" bag file, use   
`$ rosbag fix old.bag new.bag`.
