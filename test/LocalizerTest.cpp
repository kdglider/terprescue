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
#include <localizer.hpp>

/**
 * @brief      Dummy test
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(Localizer, DummyTest) {
  EXPECT_EQ(1,1);
}

TEST(Localizer, tagRecognitionNoMarkerTest) {
  std::vector<ar_track_alvar_msgs::AlvarMarker> markerList;
  Localizer tagLocalizer;
  bool noMarker = tagLocalizer.tagRecognition(markerList);
  EXPECT_EQ(noMarker, false);
}

TEST(Localizer, tagRecognitionZeroMarkerTest) {
  std::vector<ar_track_alvar_msgs::AlvarMarker> markerList;
  Localizer tagLocalizer;
  ar_track_alvar_msgs::AlvarMarker marker;
  markerList.emplace_back(marker);
  bool zeroMarker = tagLocalizer.tagRecognition(markerList);
  EXPECT_EQ(zeroMarker, true);
}

TEST(Localizer, tagRecognitionNanMarkerTest) {
  std::vector<ar_track_alvar_msgs::AlvarMarker> markerList;
  Localizer tagLocalizer;
  ar_track_alvar_msgs::AlvarMarker marker;
  marker.pose.pose.position.x = sqrt(-1);
  markerList.emplace_back(marker);
  bool nanMarker = tagLocalizer.tagRecognition(markerList);
  EXPECT_EQ(nanMarker, false);
}

TEST(Localizer, locateTagNoMarkerTest) {
  std::vector<ar_track_alvar_msgs::AlvarMarker> markerList;
  Localizer tagLocalizer;
  std::vector<tf2::Transform> tagList = tagLocalizer.locateTag(markerList);
  EXPECT_EQ(tagList.empty(), true);
}
