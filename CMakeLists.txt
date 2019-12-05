cmake_minimum_required(VERSION 2.8.3)
project(terprescue)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    tf
    cv_bridge
)
find_package(OpenCV REQUIRED)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
include_directories(vendor/googletest)

message("\n\nOPENCV VERSION:\n" ${OpenCV_INCLUDE_DIRS} "\n!!!!\n\n")

add_executable(terprescue src/main.cpp src/localizer.cpp src/terprescue.cpp)
target_link_libraries(terprescue ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(gtest)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

    add_rostest_gtest(
    terprescueTest
    test/main.cpp
    test/LocalizerTest.cpp
    test/terprescueTesting.test

    src/localizer.cpp
    )

    target_link_libraries(terprescueTest ${catkin_LIBRARIES} ${OpenCV_LIBS})

    add_dependencies(
      terprescueTest
      terprescue
      ${catkin_EXPORTED_TARGETS}
    )
endif()
