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

For exploration, TerpRescue uses the explorer class, which uses obstacle weight-based exploration to turn before reaching obstacle area.

2) 2D Map Generation

For localization and mapping, TerpRescue uses the [gmapping](http://wiki.ros.org/gmapping) ROS package, which combines LIDAR and odometry data to perform SLAM. The generated location and map are used by the explore_lite package.

3) Recognition and Localization of Packages
For QR tag recognition, TerpRescure uses the [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) package from ROS, which recognizes AR tags in an image and returns the position marker. Custom code is written which transform markers from robot frame to map frame. This location is then used to modify the map from gmapping before republishing it to be visualized in RViz.


## Personnel
Zuyang Cao: Student at the University of Maryland, Masters in Robotics

Hao Da (Kevin) Dong: Student at the University of Maryland, Masters in Systems Engineering

Jing Liang: Student at the University of Maryland, Masters in Robotics


## License
This project is licensed under the BSD 3-Clause. Please see LICENSE for additional details and disclaimer.


## Agile Iterative Process (AIP) Logs and Notes
[AIP backlogs and work log](https://drive.google.com/open?id=1RF53rFKYQvgn6KD99nCPuQfjBiVyMH979sXPxVefiFI)

[AIP sprint notes and reviews](https://drive.google.com/open?id=1kZm0ZEUZRR4xcK7r9gMdDrfFsKHa91Pvx2rPyIjw8Uw)


## Install Dependencies
``` bash
	sudo apt-get install ros-kinetic-turtlebot3-*
	sudo apt-get install ros-kinetic-gmapping
	sudo apt-get install ros-kinetic-ar-track-alvar
	sudo apt-get install ros-kinetic-rviz-visual-tools
```

## Build Instructions
``` bash
	mkdir -r terp_catkin/src && cd terp_catkin/src
	git clone https://github.com/kdglider/terprescue.git
	cd terprescue
	export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models

	cd ../..
	catkin_make
 	source devel/setup.bash
```
## Run Unit Tests
``` bash
	catkin_make run_tests
```


## Run Demonstration
In first terminal:
``` bash
	roslaunch terprescue environment_bring_up.launch
```
After gazebo and Rviz starts properly, we should be able to see the world and
turtlebot stays still at spawn position. Then, in a second terminal:
```bash
	roslaunch terprescue terprescue.launch
```

## Results
![result image](/images/terprescue.png)

[Slides](https://docs.google.com/presentation/d/13oO6MR0l_aEbyQhVjPTJkfjGrAqJLaX-ltRgInf0GAM/edit?usp=sharing) with diagrams can be found in the link.

## Notes and Known Issues
To be completed later
