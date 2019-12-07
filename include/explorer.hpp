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

#ifndef INCLUDE_EXPLORER_H_
#define INCLUDE_EXPLORER_H_

#include <vector>
#include <cmath>

/** @brief This class contains methods to help the robot navigate and explore the environment */
class Explorer {
    public:
        // Default linear and turn speeds
        double defaultLinearSpeed = 0.3;    // m/s
	    double defaultAngularSpeed = 0.8;   // rad/s

        // LIDAR distance within which the robot will execute a random turn
        double safeDistance = 0.5;          // m

        // Number of array elements in LIDAR field of view (default 0 until set in lidarCallback())
        int lidarSize = 0;

        /** @brief Constructor that initializes the first robotVelocity publish */
        Explorer();

        /**
         * @brief Checks to see if an object is close to the robot
         * @param lidarArray Array of LIDAR readings from subscribed message
         */
        bool detectObject(std::vector<float> lidarArray);

        /** @brief Executes a turn for a constrained random amount of time */
        void randomTurn();
};


#endif  // INCLUDE_EXPLORER_H_
