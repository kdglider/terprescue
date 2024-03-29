cmake_minimum_required(VERSION 2.8.3)
project(terprescue)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    ar_track_alvar_msgs
    sensor_msgs
    gazebo_msgs
    nav_msgs
    message_generation
    visualization_msgs
    tf
    cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    ar_track_alvar_msgs
    gazebo_msgs
    nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
include_directories(vendor/googletest)

message("\n\nOPENCV VERSION:\n" ${OpenCV_INCLUDE_DIRS} "\n!!!!\n\n")

add_executable(terprescue
    src/main.cpp
    src/localizer.cpp
    src/explorer.cpp
    src/terprescue.cpp
)

target_link_libraries(terprescue ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

    add_rostest_gtest(
        terprescueTest
        test/terprescueTesting.test
        test/main.cpp
        test/LocalizerTest.cpp
        test/ExplorerTest.cpp
        test/TerpRescueTest.cpp

        src/localizer.cpp
        src/terprescue.cpp
        src/explorer.cpp
    )

    target_link_libraries(terprescueTest ${catkin_LIBRARIES} ${OpenCV_LIBS})

    add_dependencies(
        terprescueTest
        terprescue
        ${catkin_EXPORTED_TARGETS}
    )
endif()
