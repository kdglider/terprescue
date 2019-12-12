/**
 * Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
 * @file       explorer.hpp
 * @date       12/06/2019
 * @brief      This file contains delarations for the Explorer class, which contains methods to help the robot navigate and explore the environment
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

#ifndef INCLUDE_EXPLORER_HPP_
#define INCLUDE_EXPLORER_HPP_

#include <iostream>
#include <vector>
#include <cmath>

/** @brief This class contains methods to help the robot navigate and
           explore the environment */
class Explorer {
 private:
        // LIDAR distance within which the robot will execute a turn
        double safeDistance = 0.8;      // m

        // Threshold within which a LIDAR reading will be used for
        // cost calculation
        double influenceThreshold = 5;  // m

        // Max LIDAR range
        double maxRange = 10;           // m

 public:
        // Number of array elements in LIDAR field of view
        // (default 0 until set in lidarCallback())
        int lidarSize = 0;

        // Array of LIDAR readings from a subscribed message
        std::vector<float> lidarArray;

        // Tolerance to consider left and right costs to be equal
        double costTolerance = 5;

        // LIDAR reading costs
        double leftCost = 0;
        double rightCost = 0;

        /**
         * @brief   Checks to see if an object is close to the robot
         * @return  True/False depending on if an object is within the safe distance
         */
        bool detectObject();

        /**
         * @brief   Calculates and updates the left and right costs of the LIDAR readings
         */
        void updateLidarCosts();
};


#endif  // INCLUDE_EXPLORER_HPP_
