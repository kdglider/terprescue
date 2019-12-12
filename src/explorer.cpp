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

#include <explorer.hpp>

bool Explorer::detectObject() {
    // Check LIDAR array for any readings below safeDistance
    for (lidarReading : lidarArray) {
        // Reject corrupt readings
        if (std::isnan(lidarReading) == false) {
            if (lidarReading < safeDistance) {
                // Return true if any reading registers an object closer than
                // safeDistance
                return true;
            }
        }
    }

    // If no object is closer than safeDistance, return false
    return false;
}


void Explorer::updateLidarCosts() {
    int lowIndex = 0;
    int highIndex = lidarSize;
    int centerIndex = lidarSize/2;

    // Reset costs
    leftCost = 0;
    rightCost = 0;

    // Sweep lidarArray for all readings and update left/right costs
    for (auto it = lidarArray.begin() ; it != lidarArray.end() ; ++it) {
        auto i = std::distance(lidarArray.begin(), it);

        if (std::isnan(*it) == false && *it < influenceThreshold) {
            if (i < centerIndex) {
                rightCost += 1/(*it);
            } else {
                leftCost += 1/(*it);
            }
        }
    }

    // std::cout << "Left Cost: " << leftCost << std::endl;
    // std::cout << "Right Cost: " << rightCost << std::endl;
}
