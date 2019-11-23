/**
 * Copyright (c) 2019   Hao Da (Kevin) Dong
 * @file       test.cpp
 * @date       2019/11/10
 * @brief      Unit tests for talker.cpp
 * @license    This project is released under the BSD-3-Clause License. See full details in LICENSE.
 */

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/printString.h"

// I have no idea why this is required for rostest/gtest to work...
std::shared_ptr<ros::NodeHandle> nh;

/**
 * @brief    Tests the output of the printString service
 */
TEST(TalkerTestSuite, transformTest) {
    // Create client to test printString service
    ros::ServiceClient testClient = nh->serviceClient<beginner_tutorials::printString>("printString");

    // Create printString service object
    beginner_tutorials::printString srv;

    // Call printString service
    srv.request.name = "Kevin";
    testClient.call(srv);

    EXPECT_EQ(srv.response.returnMsg, "This ROS service exists to serve the master: Kevin");
}

// Initialize ROS and run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "serviceTest");
    nh.reset(new ros::NodeHandle);  // Once again: WHY??
    return RUN_ALL_TESTS();
}
