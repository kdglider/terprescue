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


void TerpRescue::lidarCallback(const sensor_msgs::LaserScan msg) {
    // Calculate lidarSize if it has not been set before
    if (explorer.lidarSize == 0) {
        int lidarSize = (msg.angle_max - msg.angle_min)/msg.angle_increment;
        explorer.lidarSize = lidarSize;
        ROS_INFO_STREAM("LIDAR Size: " << lidarSize);
    }
    
    std::vector<float> lidarArray = msg.ranges;

    if (explorer.detectObject(lidarArray) == true) {
        ROS_INFO_STREAM("Object detected");
        randomTurn();
    }
}

void TerpRescue::mapCallback(const nav_msgs::OccupancyGrid data) {
  rawMap = data;
  // std::cout << rawMap.info<< std::endl;
}

void TerpRescue::arPoseCallback(const ar_track_alvar_msgs::AlvarMarkers msgs){
  markerList = msgs.markers;
}

void TerpRescue::botOdomCallback(const nav_msgs::Odometry msgs){
  botOdom = msgs;
  auto botPosition = botOdom.pose.pose.position;
  auto botOrientation = botOdom.pose.pose.orientation;
  float x = botPosition.x;
  float y = botPosition.y;
  float z = botPosition.z;
  float xQuat = botOrientation.x;
  float yQuat = botOrientation.y;
  float zQuat = botOrientation.z;
  float wQuat = botOrientation.w;
  tagWorldTransformList = tagLocalizer.transformationTagPosition(markerList, msgs);
  // std::cout<< "\ntag World tf List size: "<<tagWorldTransformList.size() <<std::endl;
  // std::cout<<"\nTurtlebot Position: " << x << ", " << y << ", " << z << std::endl;
  if(tagWorldTransformList.size() > 0){
    detectTags();
  }
}

TerpRescue::TerpRescue() {
    ros::Rate loop_rate(10);

    robotVelocity.linear.x = defaultLinearSpeed;
	robotVelocity.angular.z = 0;

    vel_pub.publish(robotVelocity);
}

void TerpRescue::randomTurn() {
    // Create and start a timer
	std::clock_t start;
	start = std::clock();
    double secondsElapsed = 0;

    // Random time (between 1-5 seconds) to turn for
    double randomSeconds = rand() % 5 + 1;

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

void TerpRescue::run(){
  // ros::spinOnce();
  // loop_rate.sleep();
}

void TerpRescue::visualization() {
}

double TerpRescue::getPointDistance(geometry_msgs::Point pointA, geometry_msgs::Point pointB){
  double distanceSquare = pow(pointA.x-pointB.x, 2) + pow(pointA.y-pointB.y, 2)
                    + pow(pointA.z-pointB.z, 2);
  double distance = sqrt(distanceSquare);
  return distance;
}

void TerpRescue::detectTags() {
  for(auto tagWorldTransform:tagWorldTransformList){
    tf2::Vector3 tagWorldTranslation = tagWorldTransform.getOrigin();
    tag tagInWorld;
    tagInWorld.ID = "tag in world frame";
    geometry_msgs::Point tagPoint;
    tagPoint.x = tagWorldTranslation.getX();
    tagPoint.y = tagWorldTranslation.getY();
    tagPoint.z = tagWorldTranslation.getZ();
    tagInWorld.tagPoint = tagPoint;
    // std::cout<<"Tag World Position: " << tagPoint.x <<", "<<tagPoint.y<<", "<<tagPoint.z;
    double minDistance = 20;
    for(auto tagItem:tagList){
      double distance = getPointDistance(tagPoint, tagItem.tagPoint);
      if(minDistance > distance){
        minDistance = distance;
      }
    }
    // std::cout<<"\nMin Distance: "<<minDistance<<std::endl;
    if(minDistance > 0.1){
      tagList.emplace_back(tagInWorld);
    }
  }
}

std::vector<ar_track_alvar_msgs::AlvarMarker> TerpRescue::getMarkerList(){
  return markerList;
}

std::vector<tf2::Transform> TerpRescue::getTagWorldTransformList(){
  return tagWorldTransformList;
}

std::vector<float> TerpRescue::getLidar() {
    return lidar;
}


// sensor_msgs::Image TerpRescue::getCameraImage() {
//     return cameraImage;
// }


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
