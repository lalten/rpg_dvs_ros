# Documentation / Report

Part of this project was contributed within the scope of a practical course at the TUM NST. This document is the final project report.

#Calibration Setup for Stereo DVS
### Practical Course

The following sections will give a precise overview of the project goal, problems and insights which we gained while developing a solution.

## Project Goal

In order to extract depth information from a camera stereo setup, most methods need some information about their positioning towards each other. The geometrical and intrinisc camera parameters as well as the rectification matrix are neccesary. The project reserach is to develop a method, how to extract the parameters form a vision stream of a stereo eDVS camera setup.

### What is an eDVS?

An eDVS (embedded Dynamic Vision Sensor) produces an event stream. Compared to a usual frame-based camera, the eDVS produces an event every time a pixel changes. This key difference enables for example very fast feedback cycles (<5ms). The used  so-called silicon retina chips are developed by the [The Institute of Neuroinformatics Zürich](https://www.ini.uzh.ch/), which also provides [further information](http://siliconretina.ini.uzh.ch/wiki/index.php). In our porject we used the miniaturized [eDVS](https://wiki.lsr.ei.tum.de/nst/programming/edvsgettingstarted).

### Software Setup

#### Existing Software as Starting Point
Camera calibration and rectification is already done routinely for "normal", frame-based cameras. Therefore, there exist many tools to tackle the task. One of them is the open-source computer vision library [OpenCV](http://opencv.org/). Based on this library, the open source [Robot Operating System (ROS)](http://wiki.ros.org/camera_calibration) provides a package, called [Camera Calibration](http://wiki.ros.org/camera_calibration). It helps to facilitate the calibration process of &bdquo;monocular or stereo cameras using a checkerboard calibration target&ldquo; <sup>[1](http://wiki.ros.org/camera_calibration)</sup>. Unfortunately, it is only for frame-based cameras. For that reason, the [Robotics and Perception Group of Zurich](http://rpg.ifi.uzh.ch/) published another open-source package for ROS called [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros). The software tries to use existing parts of the camera_calibration pacakge and OpenCV again. 

Instead of trying to reinvent the wheel again, we think the best approach is to build upon proven existing software. Therefore, this project uses the rpg_dvs_ros  package as starting point. We forked the original repository in order to implement and add our new features. 

#### eDVS Driver for ROS

The rpg_dvs_ros package (DVS_ROS) expects a rostopic input stream of the format `dvs::EventArray`. The eDVS on the other hand provides the event stream using its own custom protocol. The used communication channel is an emulated serial device over UART. Further details provides the [IniLabs eDVS guide](http://inilabs.com/support/hardware/edvs/) as well as the [Siliconretina Wiki of the INI Zurich](http://siliconretina.ini.uzh.ch/wiki/index.php). After connecting the eDVS camera over USB to a linux computer, an emulated serial device usually called `/dev/ttyUSB0` will be created. Using the console, one can send and receive commands on the interface, for example as illustrated below:

```
#set interface speed to 4Mbit
#set to 8N1 mode
$ stty -F /dev/ttyUSB0 4000000 raw

#start background process to see camera output
$ cat /dev/ttyUSB0 &

#send reset command to the camera
$ echo -ne 'R\n!' > /dev/ttyUSB0

#set event mode to 1 and start sending events
$ echo -ne '!E1\nE+\n' > /dev/ttyUSB0

#display help of available commands
$ echo -ne '??\n' > /dev/ttyUSB0

#answer could be:
EDVS128_LPC2106, V2.2: Apr 22 2015, 17:16:20  (TIMESYNC)
System Clock: 64MHz / 1us event time resolution
Supported Commands:
 E+/-       - enable/disable event sending
 !Ex        - specify event data format, ??E to list options
 !ET[=x]    - set/reset the event timestamp
 !ET[M0,M+] - active synchronized time master mode; 0:stop, +:run
 !ETS       - active synchronized time slave mode
 !Bx=y      - set bias register x[0..11] to value y[0..0xFFFFFF]
 !BF        - send bias settings to DVS
 !BDx       - select and flush default bias set (default: set 0)
 ?Bx        - get bias register x current value
 0,1,2      - LED off/on/blinking
 !S=x       - set baudrate to x
 !S[0,1,2]  - UART echo mode (none, cmd-reply, all)
 R          - reset board
 P          - enter reprogramming mode
 ??         - display help
??E
 !E0   - 2 bytes per event binary 0yyyyyyy.pxxxxxxx (default)
 !E1   - 1..3 bytes timestamp (7bits each), time difference (1us resolution)
 !E2   - 4 bytes per event (as above followed by 16bit timestamp 1us res)
 !E3   - 5 bytes per event (as above followed by 24bit timestamp 1us res)
 !E4   - 6 bytes per event (as above followed by 32bit timestamp 1us res)
 ```

> **Note** When sending multiple commands in one echo, e.g. `$ echo -ne 'R\n!S2\n!E1\nE+\n' > /dev/ttyUSB0`, the eDVS microcontroller might behave unexpected. We experienced some issues like the camera not starting to send data.

The eDVS driver for ros is based on an [eDVS.h file from NST TUM](https://wiki.lsr.ei.tum.de/nst/programming/edvs-cpp). A newly developed `edvs_ros_driver` package uses the `eDVS.h` to exchange data with the hardware. It's main purpose is the setup and integration, mainly
- reset eDVS and set proper event format
- set master and slave for time synchronization in stereo setup mode
- reset timestamps on sensors
- provide sensor information, e.g. resolution.

### Results

#### Intrinsic and Extrinsic Camera Parameters

Example of an original image vs. undistored image:
![Image](https://cdn.rawgit.com/lalten/rpg_dvs_ros/doc/doc/images/original-vs-undistored-image.svg)


### Benchmark Idea

The goal was to create an benchmark for the calibration and rectification result. Our requirements were to be reproducable and easy to perform. It should provide ground truth and according calculated depth data. This enables to calculate the 3D reproduction error.

<!-- TODO: Why not use a reference IR-marker system? -->

We propose to use a laser-cut high densitiy fiberboard construction to reproducibly move the LED board. The eDVS stereo setup is mounted on the construction. The board is moved on a z-Axis to fixed positions and snaps in. Then, the 3D position of the LEDs are measured with the eDVS. Afterwards, we calculate the distance (delta) between each 3D measurements. We also measure the ground-truth distances. Then, we compare the measured and ground-truth data in order to calculate the error. With this method we guarantee that the ground-truth data is the same for every run, even when the mounting position of the eDVS varies in the order of some millimeter.

### Learnings

#### LED Board with too many blinking LEDs
<!-- TODO: provide image of led board with all leds on -->
At the beginning we experimented with a LED board with 81 LEDs (200Hz). It produced too many events. We discovered, that the eDVS can not cope with so many events (even when under 4mBIt data rate). On the other hand, the DVS (not eDVS) has no problem with that many events. In our experience the DVS usually reports many more events from the same scene. 

#### Reflection from LED Board
<!-- TODO: provide image -->
LEDs near to the border of the board reflected too strong. They had a negative influence on the pattern detection. We experimented with some textile to absorb the reflections. The result was surpringsingly good. Yet, for our final solution, we disable the LEDs at the border to achieve the same (and do not depend on a textile). 

#### Movements during Calibration Process
Our first calibration experiments used one of the following setups: (1) The eDVS was moved around to produce events of a fixed LED board from different point of views. (2) The eDVS was fixed, while the LED board was moved around in its field of view. These approaches resulted in some issues: The subpixel accuracy calculation of the LED's center was disturbed by the moving scene. As the sensor does not produce frames, but a constant stream of events, the accumulated events of a timeframe (transition map) included a LED point, which moved. As illustrated in the figure below, the result was a visible trail of the movement in the transition map. Therefore the estimated center of the LED was not correct. This lead to suboptimal calibration results.

![Image](https://cdn.rawgit.com/lalten/rpg_dvs_ros/doc/doc/images/eDVS-moving-trail-vs-fixed.svg)


Our solution is to use both (1) a statically mounted camera and (2) a fixed mounted board. As both parts are physically not moving, we prevent the infulence of pixel trails and get better results. In order to collect image points of blinking LEDs in the hole viewing are of the sensor, the board's software moves the illuminated pattern every 10 seconds. The following picture shows the bord at the beginning and after some seconds.

![Image](https://cdn.rawgit.com/lalten/rpg_dvs_ros/doc/doc/images/led-board-moving-pattern.svg)

#### Wrong Buffering rejects Events
The buffer in the original `eDVS.h` read all avalable bytes on the serial interface. Sometimes, the buffer ended in the middle of an event package. Then, it rejected the package, because it was incomplete. Our solution was to always read at least six bytes from the serial before we try to process it. 

#### Original eDVS.h wihtout Timestamps
The original `eDVS.h` did not provide timestampts. Hence, we implemented this functionality ourselvs. As we learned later, there is an improved version available at (edvstools)](https://github.com/Danvil/edvstools).

#### Shifted X and Y Coordinates for On-Events
Sometimes the sensor image shows shifted x and y values for on-events. The reason so far is not completely clear. We used the following quick-fix (while the calibration interface was running) in a seperate terminal:
```
#press hardware reset button on eDVS

#send reset command
$ echo -ne 'R\n!' > /dev/ttyUSB0

#sometimes send reset event again, if it was not working
#(you can check if the calibration interface still shows an image)
#after the rest, it should not show any events anymore
$ echo -ne 'R\n!' > /dev/ttyUSB0

#set event mode to 1 and start sending events again
$ echo -ne '!E1\nE+\n' > /dev/ttyUSB0
#now your events should be displayed correctly
```

#### 


### Ideas for Future Improvements

#### Improve eDVS Ros Driver

There already exists an improved version of the basic eDVS.h file, which was the starting point for the ros driver. Maybe one should evaluate, weather switching to the more recent version of [the library (edvstools)](https://github.com/Danvil/edvstools) is worthwile. 

#### Buffering of Events
In order to prevent event rejection, stop parsing the buffer if less than one complete package is available. Instead, read more data into the buffer. Then, continue processing. 
