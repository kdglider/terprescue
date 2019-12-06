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

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
#include <localizer.hpp>

/**
 * @brief    recognize if there is a tag in the current image
 * @return   void
 */
bool Localizer::tagRecognition(std::vector<ar_track_alvar_msgs::AlvarMarker> markerList) {
  int markerSize = markerList.size();
  if(markerSize == 0){
    ROS_INFO_STREAM("No Tag In Sight.");
    return false;
  }
  else{
    for(auto marker : markerList){
      const geometry_msgs::PoseStamped arPoseStamped = marker.pose;
      const geometry_msgs::Pose arPose= arPoseStamped.pose;
      const geometry_msgs::Point arPoint = arPose.position;
      float x = arPoint.x;
      float y = arPoint.y;
      float z = arPoint.z;
      if(std::isnan(x) || std::isnan(y) || std::isnan(z)){
        markerSize -= 1;
      }
  }
  if(markerSize <= 0){
    ROS_INFO_STREAM("No Tag In Sight.");
    return false;
  }
  else{
    return true;
  }
  }
}
/**
 * @brief    locate tag regarding to robot frame
 * @return   void
 */
std::vector<tf2::Transform> Localizer::locateTag(std::vector<ar_track_alvar_msgs::AlvarMarker> markerList){
  std::vector<tf2::Transform> tagTransformList;
  if(tagRecognition(markerList)){
    for(auto marker : markerList){
      const geometry_msgs::PoseStamped arPoseStamped = marker.pose;
      const geometry_msgs::Pose arPose= arPoseStamped.pose;
      const geometry_msgs::Point arPoint = arPose.position;
      const geometry_msgs::Quaternion arOri = arPose.orientation;
      float x = arPoint.x;
      float y = arPoint.y;
      float z = arPoint.z;
      double xQuat = arOri.x;
      double yQuat = arOri.y;
      double zQuat = arOri.z;
      double wQuat = arOri.w;
      if(!(std::isnan(x) || std::isnan(y) || std::isnan(z))){
        tf2::Quaternion tagQuat(xQuat, yQuat, zQuat, wQuat);
        tf2::Vector3 tagVect(x, y, z);
        tf2::Transform tagTransform(tagQuat, tagVect);
        tagTransformList.emplace_back(tagTransform);
      }
    }
  }
  return tagTransformList;
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
