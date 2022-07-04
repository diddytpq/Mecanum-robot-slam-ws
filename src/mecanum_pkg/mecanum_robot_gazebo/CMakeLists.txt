cmake_minimum_required(VERSION 3.0.2)
project(mecanum_robot_gazebo)

find_package(catkin REQUIRED COMPONENTS
    gazebo_dev
    gazebo_msgs
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    tf
)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
    )
    
    include_directories(
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
    )

add_library(custom_control_plugin src/custom_control_plugin.cpp)

target_link_libraries(custom_control_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

