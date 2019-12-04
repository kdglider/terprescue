# TerpRescue Package Locator Robot
[![Build Status](https://travis-ci.com/kdglider/terprescue.svg?branch=master)](https://travis-ci.com/kdglider/terprescue)
[![Coverage Status](https://coveralls.io/repos/github/kdglider/terprescue/badge.svg?branch=master)](https://coveralls.io/github/kdglider/terprescue?branch=master)
![GitHub](https://img.shields.io/github/license/kdglider/terprescue)
---


## Overview
TerpRescue is a mobile ground robot that is able to explore/map a disaster zone and identify packages of interest. Specifically, TerpRescue is designed to recognize valuable or hazardous packages in destroyed factories/plants via QR identifier tags and report their locations to the operator along with a generated 2D map of the environment.

The prototype is based on the popular TurtleBot platform and is equipped with wheel odometers, a 2D LIDAR and an RGB camera. The concept is developed using the C++ ROS Kinetic framework and Gazebo 7.1 simulator. It is designed to operate in a simulated and bounded disaster zone environment with packages represented as cubes with QR tags.

There are three main components of the TerpRescure architecture:

1) Exploration of Disaster Environment

For exploration, TerpRescue uses the [explore_lite](http://wiki.ros.org/explore_lite) ROS package, which uses frontier-based exploration to ensure that all areas of the environment are covered by the robot.

2) 2D Map Generation

For localization and mapping, TerpRescue uses the [gmapping](http://wiki.ros.org/gmapping) ROS package, which combines LIDAR and odometry data to perform SLAM. The generated location and map are used by the explore_lite package.

3) Recognition and Localization of Packages
For QR tag recognition, TerpRescure uses the [QRCodeDetector](https://docs.opencv.org/3.4/de/dc3/classcv_1_1QRCodeDetector.html) class from OpenCV, which recognizes QR tags in an image and returns the bounding box and decoded information. Custom code is written which uses the bounding box to locate the centroid of the QR tag within the image, also taken to be the package location. This centroid is combined with LIDAR data to obtain the coordinates of the package in the robot's frame, which is then transformed into the world frame. This location is then used to modify the map from gmapping before republishing it to be visualized in RViz.


## Personnel
Zuyang Cao: Student at the University of Maryland, Masters in Robotics

Hao Da (Kevin) Dong: Student at the University of Maryland, Masters in Systems Engineering

Jing Liang: Student at the University of Maryland, Masters in Robotics


## License
This project is licensed under the BSD 3-Clause. Please see LICENSE for additional details and disclaimer. 


## Agile Iterative Process (AIP) Logs and Notes
AIP backlogs and work log:
https://drive.google.com/open?id=1RF53rFKYQvgn6KD99nCPuQfjBiVyMH979sXPxVefiFI

AIP sprint notes and reviews:
https://drive.google.com/open?id=1kZm0ZEUZRR4xcK7r9gMdDrfFsKHa91Pvx2rPyIjw8Uw


## Install Dependencies
To be completed later


## Build Instructions
```
	sudo apt-get install ros-kinetic-turtlebot3-*
```

## Run Unit Tests
To be completed later


## Run Demonstration 
To be completed later


## Notes and Known Issues
To be completed later
