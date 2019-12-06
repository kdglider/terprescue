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

Explorer::Explorer() {
    //robotVelocity.linear.x = defaultLinearSpeed;
	//robotVelocity.angular.z = 0;

    //vel_pub.publish(robotVelocity);
}

bool Explorer::detectObject(std::vector<float> lidarArray) {
    // Check LIDAR array for any readings below safeDistance
    for (int i = 1 ; i < lidarSize ; i++) {
        // Reject corrupt readings
        if (std::isnan(lidarArray[i]) == false) {
            if (lidarArray[i] < safeDistance) {
                // Return true if any reading registers an object closer than safeDistance
                return true;
                break;
            }
        }
    }

    // If no object is closer than safeDistance, return false
    return false;
}


void Explorer::randomTurn() {
    /*
    // Create and start a timer
	std::clock_t start;
	start = std::clock();
    double secondsElapsed = 0;

    // Random time (between 1-3 seconds) to turn for
    double randomSeconds = rand() % 3 + 1;

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
    */
}

