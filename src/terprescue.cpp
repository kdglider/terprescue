/**
 * Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
 * @file       terprescue.cpp
 * @date       11/23/2019
 * @brief      This class defined class of TerpRescue which is the main class of this project
 *             The class has functions to subscribe all sensor data and also calculate tag
 *             location and display map in RViz.
 * 
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
#include <iostream>


void TerpRescue::lidarCallback(const sensor_msgs::LaserScan data) {

}


void TerpRescue::cameraCallback(const sensor_msgs::Image data) {

}


void TerpRescue::odomCallback(const nav_msgs::Odometry data) {

}


void TerpRescue::mapCallback(const nav_msgs::OccupancyGrid data) {

}

void TerpRescue::arPoseCallback(const ar_track_alvar_msgs::AlvarMarkers msgs){
  markerList = msgs.markers;
  for(auto msg : markerList){
    auto arId = msg.id;
    const geometry_msgs::PoseStamped arPoseStamped = msg.pose;
    const geometry_msgs::Pose arPose= arPoseStamped.pose;
    const geometry_msgs::Point arPoint = arPose.position;
    float x = arPoint.x;
    float y = arPoint.y;
    float z = arPoint.z;
    std::cout<<"AR ID: "<<arId<<std::endl;
    std::cout<<"AR Position: "<<x<<","<<y<<","<<z<<std::endl;
  }
}

void TerpRescue::botPoseCallback(const gazebo_msgs::ModelStates msgs){
  modelStatesList = msgs;
  auto modelNameList = modelStatesList.name;
  const auto botPoseList= modelStatesList.pose; // geometry_msgs::Pose[] type
  int modelInd = 0;
  for(auto modelName:modelNameList){
    if(modelName == "turtlebot"){
      std::cout<<"Model name: " << modelName <<std::endl;
      const geometry_msgs::Point botPoint = botPoseList[modelInd].position;
      float x = botPoint.x;
      float y = botPoint.y;
      float z = botPoint.z;
      std::cout<<"Model Position: " << x << "," << y << "," << z << std::endl;
    }
  }
}

void TerpRescue::botOdomCallback(const nav_msgs::Odometry msgs){
  botOdom = msgs;
  auto botPosition = botOdom.pose.pose.position;
  auto botOrientation = botOdom.pose.pose.orientation;
  float x = botPosition.x;
  float y = botPosition.y;
  float z = botPosition.z;
  std::cout<<"Turtlebot Position: " << x << "," << y << "," << z << std::endl;
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


std::vector<TerpRescue::tag> TerpRescue::getTagList() {
    return tagList;
}
