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


void TerpRescue::lidarCallback(const sensor_msgs::LaserScan data) {

}


void TerpRescue::cameraCallback(const sensor_msgs::Image data) {

}


void TerpRescue::odomCallback(const nav_msgs::Odometry data) {

}


void TerpRescue::mapCallback(const nav_msgs::OccupancyGrid data) {

}


TerpRescue::TerpRescue() {
}


void TerpRescue::visualization() {
}


void TerpRescue::detectTags() {

}


std::vector<float> TerpRescue::getLidar() {
    return lidar;
}


sensor_msgs::Image TerpRescue::getCameraImage() {
    return cameraImage;
}


nav_msgs::OccupancyGrid TerpRescue::getRawMap() {
    return rawMap;
}


nav_msgs::OccupancyGrid TerpRescue::getSynthesizedMap() {
    return synthesizedMap;
}


geometry_msgs::Pose TerpRescue::getRobotPose() {
    return robotPose;
}


std::vector<TerpRescue::tagInfo> TerpRescue::getTagList() {
    return tagList;
}