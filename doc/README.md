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

### eDVS Driver for ROS



### Results

#### Intrinsic and Extrinsic Camera Parameters

Example of an original image vs. undistored image:
![Image](https://cdn.rawgit.com/lalten/rpg_dvs_ros/doc/doc/images/original-vs-undistored-image.svg)

Camera calibration and rectification is already done routinely for "normal", frame-based cameras. Therefore, there exist many tools to tackle the task. One of them is the open-source computer vision library [OpenCV](http://opencv.org/). Based on this library, the open source [Robot Operating System ROS](http://wiki.ros.org/camera_calibration) provides also a package, called [Camera Calibration](http://wiki.ros.org/camera_calibration).


