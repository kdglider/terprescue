# TerpRescue Package Locator Robot
[![Build Status](https://travis-ci.com/kdglider/terprescue.svg?branch=master)](https://travis-ci.com/kdglider/terprescue)
[![Coverage Status](https://coveralls.io/repos/github/kdglider/terprescue/badge.svg?branch=master)](https://coveralls.io/github/kdglider/terprescue?branch=master)
![GitHub](https://img.shields.io/github/license/kdglider/terprescue)
---


## Overview
TerpRescue is a mobile ground robot that is able to explore/map a disaster zone and identify packages of interest. Specifically, TerpRescue is designed to recognize valuable or hazardous packages in destroyed factories/plants via AR identifier tags and report their locations to the operator along with a generated 2D map of the environment.

The prototype is based on the popular TurtleBot3 platform and is equipped with wheel odometers, a 2D LIDAR and an RGB camera. The concept is developed using the C++ ROS Kinetic framework and Gazebo 7.1 simulator. It is designed to operate in a simulated and bounded disaster zone environment with packages represented as cubes with AR tags on all sides.

There are three main components of the TerpRescue architecture:

1) Exploration of Disaster Environment

For exploration, TerpRescue uses the custom Explorer class, which contains methods to detect nearby objects using the LIDAR and executing turns based on a local potential field calculated from the distance readings. See UML diagrams for more details.

2) Recognition and Localization of Packages

For AR tag recognition and localization, TerpRescure uses the [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) package from ROS, which recognizes AR tags in an image and returns the position marker. Custom code is written in the Localizer class which transforms tag coordinates from the robot frame to the world frame. The top-level TerpRescue class also includes a method to reject outliers from the positions reported. See UML diagrams for more details.

3) 2D Map Generation

For mapping, TerpRescue uses the [gmapping](http://wiki.ros.org/gmapping) ROS package, which combines LIDAR and odometry data to perform SLAM. The visualization method within the TerpRescue class takes the final list of tag postions after outlier rejection and updates the gmapping map before republishing it to be visualized in RViz.

The ROS Node and ROS Topic RQT graphs are displayed below:

![RQT image](/images/terprescue_node_rqt.png)

![RQT topic image](/images/terprescue_topic_rqt.png)


## Personnel
Zuyang Cao: Student at the University of Maryland, Masters in Robotics

Hao Da (Kevin) Dong: Student at the University of Maryland, Masters in Systems Engineering

Jing Liang: Student at the University of Maryland, Masters in Robotics


## License
This project is licensed under the BSD 3-Clause. Please see LICENSE for additional details and disclaimer.


## Agile Iterative Process (AIP) Logs and Notes
[AIP backlogs and work log](https://drive.google.com/open?id=1RF53rFKYQvgn6KD99nCPuQfjBiVyMH979sXPxVefiFI)

[AIP sprint notes and reviews](https://drive.google.com/open?id=1kZm0ZEUZRR4xcK7r9gMdDrfFsKHa91Pvx2rPyIjw8Uw)

[Presentation Slides](https://docs.google.com/presentation/d/13oO6MR0l_aEbyQhVjPTJkfjGrAqJLaX-ltRgInf0GAM/edit?usp=sharing)


## Dependencies
The system must run Ubuntu 16.04 that has C++11.

The ROS Kinetic full desktop version can be installed using these instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu

After installing ROS, use the following commands to install the other necessary packages:
``` bash
	sudo apt-get install ros-kinetic-turtlebot3-*
	sudo apt-get install ros-kinetic-gmapping
	sudo apt-get install ros-kinetic-ar-track-alvar
	sudo apt-get install ros-kinetic-rviz-visual-tools
```

## Build Instructions
Create a new directory on the local system to designate as the Catkin workspace and create another directory called "src" within it (eg. catkin_ws/src). Clone the project into the src directory:
``` bash
    git clone https://github.com/kdglider/terprescue.git
```

From the catkin_ws/src directory, change directory into the terprescue repository and update the GAZEBO_MODEL_PATH to include our models:
``` bash
	cd terprescue
	export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models
```

Change directory back up to catkin_ws, build the code and run the setup script:
``` bash
	cd ../..
	catkin_make
 	source devel/setup.bash
```


## Run Unit Tests
To run our unit tests (optional), append run_tests after the catkin_make command:
``` bash
	catkin_make run_tests
```


## Run Demonstration
To run the demonstration with our sample environment, open two terminals. In the first terminal, launch the Gazebo environment:
``` bash
	roslaunch terprescue environment_bring_up.launch
```

After Gazebo and Rviz finishes loading properly, launch the terprescue node and gmapping in the second terminal:
```bash
	roslaunch terprescue terprescue.launch
```

The robot will begin exploration and a map will be generated in RViz with the package locations as red dots.


## Results
The image below displays a sample run of our demonstration. The red dots are encased by square black borders, which represents the sides of the cubes (packages) detected by the LIDAR.

![result image](/images/terprescue.png)


## Notes and Known Issues
1) The custom exploration algorithm requires extensive tuning of parameters within the Explorer class, but due to the design of the algorithm, it is still possible for the robot to become stuck (turning back and forth indefinitely) during certain edge cases within the environment. Tuning can mitigate this issue, but a better algorithm can also be employed as an alternative.

2) The ar_track_alvar detector is known to collect a large amount of false positives. Our rejectTagOutliers method within the TerpRescue class aims to eliminate as many of these as possible before visualization, but it is possible to develop a more sophisticaled filtering/clustering algorithm to improve results.
