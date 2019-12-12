/**
 * Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
 * @file       localizer.cpp
 * @date       11/23/2019
 * @brief      This class defined class of localizer which is the class calculating tags positions
 *             And also recognize tag ID
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

#include <localizer.hpp>

bool Localizer::recognizeTag(
    std::vector<ar_track_alvar_msgs::AlvarMarker> markerList) {

    int markerSize = markerList.size();

    // If markerList is not empty, reject error readings
    if (markerSize == 0) {
        ROS_INFO_STREAM("No Tag In Sight.");
        return false;
    } else {
        for (auto marker : markerList) {
            const geometry_msgs::PoseStamped arPoseStamped = marker.pose;
            const geometry_msgs::Pose arPose = arPoseStamped.pose;
            const geometry_msgs::Point arPoint = arPose.position;
            float x = arPoint.x;
            float y = arPoint.y;
            float z = arPoint.z;
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
                markerSize -= 1;
            }
        }

        // Return true if markerList is still not empty
        if (markerSize <= 0) {
            ROS_INFO_STREAM("No Tag In Sight.");
            return false;
        } else {
            return true;
        }
    }
}


std::vector<tf2::Transform> Localizer::locateTag(
    std::vector <ar_track_alvar_msgs::AlvarMarker> markerList) {

    // Create new tagTransformList to return
    std::vector<tf2::Transform> tagTransformList;

    // Create new Transform objects from markers and add to tagTransformList
    if (recognizeTag(markerList)) {
        for (auto marker : markerList) {
            const geometry_msgs::PoseStamped arPoseStamped = marker.pose;
            const geometry_msgs::Pose arPose = arPoseStamped.pose;
            const geometry_msgs::Point arPoint = arPose.position;
            const geometry_msgs::Quaternion arOri = arPose.orientation;

            float x = arPoint.x;
            float y = arPoint.y;
            float z = arPoint.z;

            double xQuat = arOri.x;
            double yQuat = arOri.y;
            double zQuat = arOri.z;
            double wQuat = arOri.w;

            if (!(std::isnan(x) || std::isnan(y) || std::isnan(z))) {
                tf2::Quaternion tagQuat(xQuat, yQuat, zQuat, wQuat);
                tf2::Vector3 tagVect(x, y, z);
                tf2::Transform tagTransform(tagQuat, tagVect);
                tagTransformList.emplace_back(tagTransform);
            }
        }
    }

    return tagTransformList;
}


std::vector<tf2::Transform> Localizer::transformationTagPosition(
    const std::vector<ar_track_alvar_msgs::AlvarMarker> &markerList,
    const nav_msgs::Odometry odomMsg) {

    // Create new tagWorldTransformList to return
    std::vector<tf2::Transform> tagWorldTransformList;

    // Get current tagTransformList using locateTag()
    std::vector<tf2::Transform> tagTransformList;
    tagTransformList = locateTag(markerList);

    // Transform tag coordinates to world frame using robot odometry data
    if (tagTransformList.size() > 0) {
        auto botPosition = odomMsg.pose.pose.position;
        auto botOrientation = odomMsg.pose.pose.orientation;

        float x = botPosition.x;
        float y = botPosition.y;
        float z = botPosition.z;

        double xQuat = botOrientation.x;
        double yQuat = botOrientation.y;
        double zQuat = botOrientation.z;
        double wQuat = botOrientation.w;

        tf2::Quaternion botQuat(xQuat, yQuat, zQuat, wQuat);
        tf2::Vector3 botVect(x, y, z);
        tf2::Transform botTransform(botQuat, botVect);

        for (auto tagTransform : tagTransformList) {
            tf2::Transform tagWorldTransform = botTransform*tagTransform;
            tagWorldTransformList.emplace_back(tagWorldTransform);
        }
    }

    return tagWorldTransformList;
}
