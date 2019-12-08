/**Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
 * @file       localizer.hpp
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

#ifndef INCLUDE_LOCALIZER_H_
#define INCLUDE_LOCALIZER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <vector>

/**
 * @brief The class has can use input data of lidar, camera and also robot's frame to
 *        recognize tags and get tags' location and ID
 */
class Localizer {
    public:
        /**
         * @brief   Recognize if there is a tag in the current image
         * @param   markerList List of AR marker messages
         * @return  True/False depending on if there is an AR tag in the current camera frame
         */
        bool recognizeTag(std::vector<ar_track_alvar_msgs::AlvarMarker> markerList);

        /**
         * @brief   Localize AR tag with respect to robot frame
         * @param   markerList List of AR marker messages
         * @return  List of tf2::Transform objects that represent each tag's pose in the robot frame
         */
        std::vector<tf2::Transform> locateTag(std::vector<ar_track_alvar_msgs::AlvarMarker> markerList);

        /**
         * @brief   Tranform tag locations from robot frame to world frame
         * @param   markerList List of AR marker messages
         * @param   odomMsg Contains the current pose of robot
         * @return  List of tf2::Transform objects that represent each tag's pose in the world frame
         */
        std::vector<tf2::Transform> transformationTagPosition(std::vector<ar_track_alvar_msgs::AlvarMarker> markerList, const nav_msgs::Odometry odomMsg);
};

#endif  // INCLUDE_LOCALIZER_H_
