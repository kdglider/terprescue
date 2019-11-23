/**
 * Copyright (c) 2019   Hao Da (Kevin) Dong
 * @file       walker.cpp
 * @date       2019/11/16
 * @brief      Implementations for the Walker class
 * @license    This project is released under the BSD-3-Clause License. See full details in LICENSE.
 */

#include <walker.h>

Walker::Walker() {
    robotVelocity.linear.x = defaultLinearSpeed;
	robotVelocity.angular.z = 0;

    vel_pub.publish(robotVelocity);
}

void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Number of array elements in laser field of view
    int laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;

    // Check LIDAR array for any readings below safeDistance
    for (int i = 1 ; i < laserSize ; i++) {
        // Reject corrupt readings
        if (std::isnan(msg->ranges[i]) == false) {
            if (msg->ranges[i] < safeDistance) {
                randomTurn();
                break;
            }
        }
    }

    // If no obstacle is closer than safeDistance, continue to move forward
    vel_pub.publish(robotVelocity);
}


void Walker::randomTurn() {
    // Create and start a timer
	std::clock_t start;
	start = std::clock();
    double secondsElapsed = 0;

    // Random time (between 1-3 seconds) to turn for
    double randomSeconds = rand() % 3 + 1;

    // Change velcity profile to turning
    robotVelocity.linear.x = 0;
	robotVelocity.angular.z = defaultAngularSpeed;

    // Keep turning until the random time is reached
	while (secondsElapsed <= randomSeconds) {
		vel_pub.publish(robotVelocity);
		secondsElapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
	}

    // Change velocity profile back to moving forward
    robotVelocity.linear.x = defaultLinearSpeed;
	robotVelocity.angular.z = 0;
    vel_pub.publish(robotVelocity);
}

