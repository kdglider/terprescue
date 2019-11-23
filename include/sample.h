/**
 * Copyright (c) 2019   Hao Da (Kevin) Dong
 * @file       walker.h
 * @date       2019/11/16
 * @brief      Declarations for the Walker class
 * @license    This project is released under the BSD-3-Clause License. See full details in LICENSE.
 */

#ifndef INCLUDE_WALKER_H_
#define INCLUDE_WALKER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>
#include <ctime>
#include <sstream>

/**
 * @brief printString service callback function that returns a sentence with the name given in the request
 * @param req Service request
 * @param res Service response
 * @return None
 */
class Walker {
    public:
        ros::NodeHandle nh;

        /** @brief Subscriber for 2D LIDAR scan */
        ros::Subscriber laser_sub = nh.subscribe("scan", 10, &Walker::laserCallback, this);

        // Publisher for mobile base velocity
        ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

        // Robot velocity message to be published
        geometry_msgs::Twist robotVelocity;

        // Default linear and turn speeds
        double defaultLinearSpeed = 0.3;      // m/s
	    double defaultAngularSpeed = 0.8;     // rad/s

        // LIDAR distance within which the robot will execute a random turn
        double safeDistance = 0.5;        // m

        /** @brief Constructor that initializes the first robotVelocity publish */
        Walker();

        /**
         * @brief LIDAR callback function that determines if the robot is too close to an obstacle
         * @param msg LaserScan message that contains an array with the depth readings
         */
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

        /** @brief Executes a turn for a constrained random amount of time */
        void randomTurn();

};


#endif  // INCLUDE_WALKER_H_
