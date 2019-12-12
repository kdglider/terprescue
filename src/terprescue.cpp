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

TerpRescue::TerpRescue() {
    // Set publishing rate to 10 Hz
    ros::Rate loop_rate(10);

    // Set and publish initial robot velocity to move forward
    robotVelocity.linear.x = defaultLinearSpeed;
    robotVelocity.angular.z = 0;

    vel_pub.publish(robotVelocity);
}


void TerpRescue::lidarCallback(const sensor_msgs::LaserScan msg) {
    // Calculate lidarSize if it has not been set before
    if (explorer.lidarSize == 0) {
        int lidarSize = (msg.angle_max - msg.angle_min)/msg.angle_increment;
        explorer.lidarSize = lidarSize;
    }

    // Update LIDAR left and right costs
    explorer.lidarArray = msg.ranges;
    explorer.updateLidarCosts();

    // Check if the total cost exceeds a threshold
    // This helps the robot get out of corners
    if (explorer.leftCost + explorer.rightCost > 135) {
        robotVelocity.linear.x = 0;
        robotVelocity.angular.z = defaultAngularSpeed;
        vel_pub.publish(robotVelocity);

    // Check if an object is within the safe distance or if
    // either the left/right costs are too high
    } else if (explorer.detectObject() == true || explorer.leftCost > 80 ||
               explorer.rightCost > 80) {
        // Change velcity profile to turn left or right
        // depending on the LIDAR costs
        if (explorer.leftCost < explorer.rightCost) {
            robotVelocity.linear.x = 0;
            robotVelocity.angular.z = defaultAngularSpeed;
        } else {
            robotVelocity.linear.x = 0;
            robotVelocity.angular.z = -defaultAngularSpeed;
        }

        vel_pub.publish(robotVelocity);

    } else {
        // Change velocity profile back to moving forward
        robotVelocity.linear.x = defaultLinearSpeed;
        robotVelocity.angular.z = 0;
        vel_pub.publish(robotVelocity);
    }
}


void TerpRescue::mapCallback(const nav_msgs::OccupancyGrid data) {
    // Update rawMap with new data from gmapping
    rawMap = data;

    // Update synthesized map
    visualization();
}


void TerpRescue::arPoseCallback(const ar_track_alvar_msgs::AlvarMarkers msgs) {
    // Update markerList with current AR markers in camera frame
    markerList = msgs.markers;
}


void TerpRescue::botOdomCallback(const nav_msgs::Odometry msgs) {
    // Update botOdom with new subscribed message
    botOdom = msgs;
    auto botPosition = botOdom.pose.pose.position;
    auto botOrientation = botOdom.pose.pose.orientation;

    // Localize tags in current markerList
    tagWorldTransformList =
        tagLocalizer.transformationTagPosition(markerList, msgs);

    // Reject outliers if the localized tag list is not empty
    if (tagWorldTransformList.size() > 0) {
        rejectTagOutliers();
    }
}


void TerpRescue::visualization() {
    if (tagList.size() > 0) {
        std::cout << tagList.size() << std::endl;
        tagMarkers.markers.clear();
        tagPublisher.publish(tagMarkers);

        // Create new visualization markers for each tag in tagList
        for (auto tag : tagList) {
            if (tag.positionCount < 75) {
              continue;
            }
            visualization_msgs::Marker tagMarker;
            tagMarker.header.frame_id = "map";
            tagMarker.header.stamp = ros::Time();
            tagMarker.id = tag.ID;
            tagMarker.type = visualization_msgs::Marker::SPHERE;
            tagMarker.pose.position = tag.tagPoint;
            tagMarker.action = visualization_msgs::Marker::ADD;
            tagMarker.color.a = 1;
            tagMarker.color.r = 1;
            tagMarker.scale.x = 0.5;
            tagMarker.scale.y = 0.5;
            tagMarkers.markers.push_back(tagMarker);
        }

        // Publish visualization markers
        tagPublisher.publish(tagMarkers);

    } else {
        ROS_INFO_STREAM("No tag detected!");
    }
}


double TerpRescue::getPointDistance(
    geometry_msgs::Point pointA,
    geometry_msgs::Point pointB) {

    // Calculate Euclidean distance
    double distanceSquare = pow(pointA.x-pointB.x, 2) +
                            pow(pointA.y-pointB.y, 2) +
                            pow(pointA.z-pointB.z, 2);
    double distance = sqrt(distanceSquare);

    return distance;
}


void TerpRescue::rejectTagOutliers() {
    for (auto tagWorldTransform : tagWorldTransformList) {
        tf2::Vector3 tagWorldTranslation = tagWorldTransform.getOrigin();
        tag tagInWorld;
        tagInWorld.ID = tagList.size();
        tagInWorld.positionCount = 0;

        // Create tagPoint to perform comparisons
        geometry_msgs::Point tagPoint;
        tagPoint.x = tagWorldTranslation.getX();
        tagPoint.y = tagWorldTranslation.getY();
        tagPoint.z = tagWorldTranslation.getZ();

        // Reject tagPoint if certain values exceed thresholds
        if (tagPoint.z < 0 || tagPoint.z > 0.8) {
            continue;
        }
        if (tagPoint.x < -1.5 || tagPoint.x > 10) {
            continue;
        }
        if (tagPoint.y < -8 || tagPoint.y > 1.5) {
            continue;
        }

        // Set all z values to be the same since it is a 2D map
        tagPoint.z = 0.1;

        // Compare tagPoint to all current tags in tagList
        // Update position if it belongs to an existing package
        tagInWorld.tagPoint = tagPoint;
        double minDistance = 20;
        for (auto &tagItem : tagList) {
            double distance = getPointDistance(tagPoint, tagItem.tagPoint);
            if (minDistance > distance) {
                minDistance = distance;
            }
            if (distance <= 2) {
                tagItem.positionCount += 1;
                tagItem.tagPoint.x = (tagItem.tagPoint.x *
                                      tagItem.positionCount +
                                      tagPoint.x)/(tagItem.positionCount + 1);
                tagItem.tagPoint.y = (tagItem.tagPoint.y *
                                      tagItem.positionCount +
                                      tagPoint.y)/(tagItem.positionCount + 1);
            }
        }

        // If tagPoint is far away from other tags, treat it as a new tag
        if (minDistance > 2) {
            tagList.emplace_back(tagInWorld);
        }
    }
}


std::vector<ar_track_alvar_msgs::AlvarMarker> TerpRescue::getMarkerList() {
    return markerList;
}


std::vector<tf2::Transform> TerpRescue::getTagWorldTransformList() {
    return tagWorldTransformList;
}


std::vector<TerpRescue::tag> TerpRescue::getTagList() {
    return tagList;
}
