/**Copyright (c) 2019 Jing Liang, Kevin Dong, Zuyang Cao
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

/**
 * @brief    recognize if there is a tag in the current image
 * @return   void
 */
void Localizer::tagRecognition() {
}

/**
 * @brief    locate tag regarding to robot frame
 * @return   void
 */
void Localizer::locateTag() {

}

/**
 * @brief    tranform tag location from robot frame to map frame
 * @return   void
 */
void Localizer::transformationTagPosition() {

}

/**
 * @brief    This function detects tags and also save tags information in a vector
 * @param    lidar data: vector<float>
 * @param    image data: sensor_msgs::Image
 * @param    robot's currentLocation: geometry_msgs::Pose
 * @return   void
 */
void Localizer::tagDetection(sensor_msgs::Image image, std::vector<float> lidar, geometry_msgs::Pose currentLocation) {

}

/**
 * @brief    return current detected tags' information
 * @return   a vector of tags' information
 */
std::vector<Localizer::tagInfo> Localizer::getTagInfo() {
    tagInfo tag;
    tags.push_back(tag);
    return tags;
}