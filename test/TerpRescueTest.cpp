/** Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
 *  @file       LocalizerTest.cpp
 *  @brief      Localizer class test file
 *  @license    BSD 3-Clause LICENSE
 *
 * Copyright (c) 2018, Zuyang Cao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <terprescue.hpp>

/**
 * @brief      Dummy test
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(TerpRescue, DummyTest) {
  EXPECT_EQ(1,1);
}

TEST(TerpRescue, arPoseCallbackTest) {
  ros::NodeHandle nh;
  ros::Publisher testPub = nh.advertise<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 50);
  ar_track_alvar_msgs::AlvarMarkers markersMsg;
  ar_track_alvar_msgs::AlvarMarker marker;
  std::vector<ar_track_alvar_msgs::AlvarMarker> markerList;
  marker.pose.pose.position.x = 1;
  marker.pose.pose.position.y = 1;
  marker.pose.pose.position.z = 1;
  markerList.emplace_back(marker);
  markersMsg.markers = markerList;
  TerpRescue terpRescue;
  int counter = 0;
  while (ros::ok()) {
    testPub.publish(markersMsg);
    terpRescue.run();
    if (counter == 2) {
      break;
    }
    ros::spinOnce();
    counter++;
  }
  std::vector<ar_track_alvar_msgs::AlvarMarker> markerListTest = terpRescue.getMarkerList();
  EXPECT_EQ(marker.pose.pose.position.x, markerListTest[0].pose.pose.position.x);
}

TEST(TerpRescue, botOdomCallbackTest) {
  ros::NodeHandle nh;
  ros::Publisher testPub = nh.advertise<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 50);
  ros::Publisher testOdomPub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
  ar_track_alvar_msgs::AlvarMarkers markersMsg;
  ar_track_alvar_msgs::AlvarMarker marker;
  nav_msgs::Odometry OdomMsgs;
  OdomMsgs.pose.pose.position.x = 1;
  OdomMsgs.pose.pose.position.y = 1;
  OdomMsgs.pose.pose.position.z = 1;
  std::vector<ar_track_alvar_msgs::AlvarMarker> markerList;
  marker.pose.pose.position.x = 1;
  marker.pose.pose.position.y = 1;
  marker.pose.pose.position.z = 1;
  markerList.emplace_back(marker);
  markersMsg.markers = markerList;
  TerpRescue terpRescue;
  int counter = 0;
  while (ros::ok()) {
    testPub.publish(markersMsg);
    testOdomPub.publish(OdomMsgs);
    terpRescue.run();
    if (counter == 2) {
      break;
    }
    ros::spinOnce();
    counter++;
  }
  auto tagList = terpRescue.getTagWorldTransformList();
  EXPECT_EQ(tagList.size(), 1);
}

TEST(TerpRescue, getPointDistanceTest) {
  geometry_msgs::Point pointA;
  geometry_msgs::Point pointB;
  pointA.x = 1;
  pointA.y = 1;
  pointA.z = 1;
  pointB.x = 1;
  pointB.y = 1;
  pointB.z = 1;
  TerpRescue terpRescue;
  auto distance = terpRescue.getPointDistance(pointA, pointB);
  EXPECT_EQ(distance, 0);
}

TEST(TerpRescue, detectTagsTest) {
  ros::NodeHandle nh;
  ros::Publisher testPub = nh.advertise<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 50);
  ros::Publisher testOdomPub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
  ar_track_alvar_msgs::AlvarMarkers markersMsg;
  ar_track_alvar_msgs::AlvarMarker marker;
  nav_msgs::Odometry OdomMsgs;
  OdomMsgs.pose.pose.position.x = 1;
  OdomMsgs.pose.pose.position.y = 1;
  OdomMsgs.pose.pose.position.z = 1;
  std::vector<ar_track_alvar_msgs::AlvarMarker> markerList;
  marker.pose.pose.position.x = 1;
  marker.pose.pose.position.y = 1;
  marker.pose.pose.position.z = 1;
  markerList.emplace_back(marker);
  markersMsg.markers = markerList;
  TerpRescue terpRescue;
  int counter = 0;
  while (ros::ok()) {
    testPub.publish(markersMsg);
    testOdomPub.publish(OdomMsgs);
    terpRescue.run();
    if (counter == 2) {
      break;
    }
    ros::spinOnce();
    counter++;
  }
  auto tagList = terpRescue.getTagList();
  EXPECT_EQ(tagList.size(), 2);
}
