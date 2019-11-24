/**Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
 * @file       terprescue.cpp
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

#include <terprescue.hpp>

/**
 * @brief    callback function of lidar
 * @param    lidar data: sensor_msgs::LaserScan
 * @return   void
 */
void TerpRescue::lidarCallback(const sensor_msgs::LaserScan data) {

}

/**
 * @brief    callback function of camera
 * @param    lidar data: sensor_msgs::Image
 * @return   void
 */
void TerpRescue::cameraCallback(const sensor_msgs::Image data) {

}

/**
 * @brief    callback function of odom
 * @param    lidar data: nav_msgs::Odometry
 * @return   void
 */
void TerpRescue::odomCallback(const nav_msgs::Odometry data) {

}

/**
 * @brief    callback function of map
 * @param    lidar data: nav_msgs::OccupancyGrid
 * @return   void
 */
void TerpRescue::mapCallback(const nav_msgs::OccupancyGrid data) {

}


/**
 * @brief    Constructor of the class which initialize parameters
 */
TerpRescue::TerpRescue() {
}

/**
 * @brief    display synthesized map in rviz
 * @return   void
 */
void TerpRescue::visualization() {
}

/**
 * @brief    use sensor datas to detect tags and get their locations
 * @return   void
 */
void TerpRescue::detectTags() {

}

/**
 * @brief    return current lidar data
 * @return   lidar data: vector<float>
 */
std::vector<float> TerpRescue::getLidar() {
    std::vector<float> lidar;
    return lidar;
}

/**
 * @brief    return current image data
 * @return   camera data: sensor_msgs::Image
 */
sensor_msgs::Image TerpRescue::getCamera() {
    sensor_msgs::Image image;
    return image;
}

/**
 * @brief    return current map data from gmapping
 * @return   map data: nav_msgs::OccupancyGrid
 */
nav_msgs::OccupancyGrid TerpRescue::getRawMap() {
    nav_msgs::OccupancyGrid grid;
    return grid;
}

/**
 * @brief    return synthesized map data include detected tags
 * @return   synthesized map data: nav_msgs::OccupancyGrid
 */
nav_msgs::OccupancyGrid TerpRescue::getSynthesizedMap() {
    nav_msgs::OccupancyGrid grid;
    return grid;
}

/**
 * @brief    return current pose of robot
 * @return   current robot's location data: geometry_msgs::Pose
 */
geometry_msgs::Pose TerpRescue::getCurrentLocation() {
    geometry_msgs::Pose pose;
    return pose;
}

/**
 * @brief    return current detected tags' information
 * @return   all current detected tags: vector of struct
 */
std::vector<TerpRescue::tagInfo> TerpRescue::getTagInformation() {
    TerpRescue::tagInfo tag;
    tags.push_back(tag);
    return tags;
}