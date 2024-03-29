/**
 * Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
 * @file       terprescue.hpp
 * @date       11/23/2019
 * @brief      This class defined class of TerpRescue which is the main class of this project
 *             The class has functions to subscribe all sensor data and also calculate tag
 *             location and display map in rviz.
 * @license    This project is released under the BSD-3-Clause License.
 *             Redistribution and use in source and binary forms, with or without
 *             modification, are permitted provided that the following conditions are met:
 *
 *             1. Redistributions of source code must retain the above copyright notice, this
 *                list of conditions and the following disclaimer.
 *
 *             2. Redistributions in binary form must reproduce the above copyright notice,
 *                this list of conditions and the following disclaimer in the documentation
 *                and/or other materials provided with the distribution.
 *
 *             3. Neither the name of the copyright holder nor the names of its
 *                contributors may be used to endorse or promote products derived from
 *                this software without specific prior written permission.
 *
 *             THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *             AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *             IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *             DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *             FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *             DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *             SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *             CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *             OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *             OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDE_TERPRESCUE_HPP_
#define INCLUDE_TERPRESCUE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_msgs/ModelStates.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/tokenizer.hpp>
*/

#include <vector>
#include <string>
#include <cmath>
#include <ctime>

#include <localizer.hpp>
#include <explorer.hpp>

/**
 * @brief The class has functions to subscribe all sensor data and also calculate tag
 *             location and display map in RViz.
 */
class TerpRescue {
 private:
        ros::NodeHandle nh;

        // Structure of a tag; contains the ID and pose
        struct tag{
            int ID;                             // Decoded tag ID
            int positionCount;
            geometry_msgs::Point tagPoint;      // Tag pose in the world frame
        };

        std::vector<tag> tagList;       // List of all located tags (packages)

        geometry_msgs::Pose robotPose;      // Current robot pose

        nav_msgs::OccupancyGrid rawMap;     // Raw map from gmapping

        // Synthesized map with package locations
        nav_msgs::OccupancyGrid synthesizedMap;

        visualization_msgs::MarkerArray tagMarkers;

        std::vector<float> lidar;           // LIDAR data

        // AR markers list data
        std::vector<ar_track_alvar_msgs::AlvarMarker> markerList;

        nav_msgs::Odometry botOdom;         // Turtlebot Odometry information

        // Robot velocity message to be published
        geometry_msgs::Twist robotVelocity;

        // Default linear and turn speeds
        double defaultLinearSpeed = 0.36;       // m/s
        double defaultAngularSpeed = 0.42;      // rad/s

        Localizer tagLocalizer;             // Instantiate a Localizer object

        Explorer explorer;                  // Instantiate an Explorer object

        // LIDAR subscriber
        ros::Subscriber lidarSubscriber = nh.subscribe<sensor_msgs::LaserScan>
        ("/scan", 1, &TerpRescue::lidarCallback, this);
        std::vector<tf2::Transform> tagWorldTransformList;

        // AR tag subscriber
        ros::Subscriber arSubscriber = nh.subscribe<ar_track_alvar_msgs::
        AlvarMarkers>("/ar_pose_marker", 50, &TerpRescue::arPoseCallback, this);

        // Odometry subscriber
        ros::Subscriber odomSubscriber = nh.subscribe<nav_msgs::Odometry>
        ("/odom", 50, &TerpRescue::botOdomCallback, this);

        // Raw map subscriber
        ros::Subscriber mapSubscriber = nh.subscribe<nav_msgs::OccupancyGrid>
        ("/map", 1, &TerpRescue::mapCallback, this);

        // Synthesized map publisher
        ros::Publisher tagPublisher = nh.advertise<visualization_msgs::
        MarkerArray>("/tagsMarker", 10);

        // Publisher for mobile base velocity
        ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
        ("/cmd_vel", 1, this);


        /**
         * @brief   LIDAR callback function
         * @param   msg sensor_msgs::LaserScan
         * @return  void
         */
        void lidarCallback(const sensor_msgs::LaserScan msg);

        /**
         * @brief   Map callback function
         * @param   data nav_msgs::OccupancyGrid
         * @return  void
         */
        void mapCallback(const nav_msgs::OccupancyGrid data);

        /**
         * @brief   Odometry callback function
         * @param   msgs nav_msgs::Odometry
         * @return  void
         */
        void botOdomCallback(const nav_msgs::Odometry msgs);

        /**
         * @brief   AR tag callback function
         * @param   msgs ar_track_alvar_msgs::AlvarMarkers
         * @return  void
         */
        void arPoseCallback(const ar_track_alvar_msgs::AlvarMarkers msgs);

 public:
        /**
         * @brief   Constructor of the class which published initial robot velocity
         */
        TerpRescue();

        /**
         * @brief   Calculates the Euclidean distance between two points
         * @param   pointA First point
         * @param   pointB Second point
         * @return  Euclidean distance as a double
         */
        double getPointDistance(
            geometry_msgs::Point pointA,
            geometry_msgs::Point pointB);

        /**
         * @brief   Displays the synthesized map in RViz
         * @return  void
         */
        void visualization();

        /**
         * @brief   Reject outlier tag positions from tagWorldTransformList
         * @return  void
         */
        void rejectTagOutliers();

        /**
         * @brief   Return current tag list
         * @return  tagList
         */
        std::vector<tag> getTagList();

        /**
         * @brief   Return current markerList
         * @return  markerList
         */
        std::vector<ar_track_alvar_msgs::AlvarMarker> getMarkerList();

        /**
         * @brief   Return current tag transform in world frame
         * @return  tagWorldTransformList
         */
        std::vector<tf2::Transform> getTagWorldTransformList();
};


#endif  // INCLUDE_TERPRESCUE_HPP_
