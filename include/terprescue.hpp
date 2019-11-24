/**Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
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

#ifndef INCLUDE_TERPRESCUE_H_
#define INCLUDE_TERPRESCUE_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <localizer.hpp>

/**
 * @brief The class has functions to subscribe all sensor data and also calculate tag
 *             location and display map in rviz.
 */
class TerpRescue {
 private:
    ros::NodeHandle nh;
    nav_msgs::OccupancyGrid rawMap;
    nav_msgs::OccupancyGrid synthesizedMap;

    // tagInfo is the information of each tag, which include tag id and position
    struct tagInfo {
        int ID;
        geometry_msgs::Pose position;
    };

    std::vector<tagInfo> tags;  // vector of all tags information
    std::vector<float> lidar;
    sensor_msgs::Image camera;

    Localizer tagLocalizer;  // created instance of localizer to detect tag's location

    // lidar subscriber
    ros::Subscriber lidarSubscriber = nh.subscribe<sensor_msgs::LaserScan>("/scan_filtered", 
        1, &TerpRescue::lidarCallback, this);

    // camera subscriber
    ros::Subscriber cameraSubscriber = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 
        1, &TerpRescue::cameraCallback, this);

    // odom subscriber
    ros::Subscriber odomSubscriber = nh.subscribe<nav_msgs::Odometry>("/camera/depth/image_raw", 
        1, &TerpRescue::odomCallback, this);

    // map subscriber
    ros::Subscriber mapSubscriber = nh.subscribe<nav_msgs::OccupancyGrid>("/camera/depth/image_raw", 
        1, &TerpRescue::mapCallback, this);

    // publisher to publish map
    ros::Publisher mapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("/synthesizedmap", 10);


    /**
     * @brief    callback function of lidar
     * @param    lidar data: sensor_msgs::LaserScan
     * @return   void
     */
    void lidarCallback(const sensor_msgs::LaserScan data);

    /**
     * @brief    callback function of camera
     * @param    lidar data: sensor_msgs::Image
     * @return   void
     */
    void cameraCallback(const sensor_msgs::Image data);

    /**
     * @brief    callback function of odom
     * @param    lidar data: nav_msgs::Odometry
     * @return   void
     */
    void odomCallback(const nav_msgs::Odometry data);

    /**
     * @brief    callback function of map
     * @param    lidar data: nav_msgs::OccupancyGrid
     * @return   void
     */
    void mapCallback(const nav_msgs::OccupancyGrid data);

 public:
    /**
     * @brief    Constructor of the class which initialize parameters
     */
    TerpRescue();

    /**
     * @brief    display synthesized map in rviz
     * @return   void
     */
    void visualization();

    /**
     * @brief    use sensor datas to detect tags and get their locations
     * @return   void
     */
    void detectTags();

    /**
     * @brief    return current lidar data
     * @return   lidar data: vector<float>
     */
    std::vector<float> getLidar();

    /**
     * @brief    return current image data
     * @return   camera data: sensor_msgs::Image
     */
    sensor_msgs::Image getCamera();

    /**
     * @brief    return current map data from gmapping
     * @return   map data: nav_msgs::OccupancyGrid
     */
    nav_msgs::OccupancyGrid getRawMap();

    /**
     * @brief    return synthesized map data include detected tags
     * @return   synthesized map data: nav_msgs::OccupancyGrid
     */
    nav_msgs::OccupancyGrid getSynthesizedMap();

    /**
     * @brief    return current pose of robot
     * @return   current robot's location data: geometry_msgs::Pose
     */
    geometry_msgs::Pose getCurrentLocation();

    /**
     * @brief    return current detected tags' information
     * @return   all current detected tags: vector of struct
     */
    std::vector<tagInfo> getTagInformation();
};


#endif  // INCLUDE_TERPRESCUE_H_
