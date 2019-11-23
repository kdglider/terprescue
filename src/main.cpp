/**
 * Copyright (c) 2019   Hao Da (Kevin) Dong
 * @file       main.cpp
 * @date       2019/11/16
 * @brief      Main file for the TurtleBot random walker
 * @license    This project is released under the BSD-3-Clause License. See full details in LICENSE.
 */

#include <walker.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");

    Walker walker;

    ros::spin();

    return 0;
}
