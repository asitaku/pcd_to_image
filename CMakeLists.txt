cmake_minimum_required(VERSION 3.0.2)
project(pcd_to_image)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosmsg
        rospy
        nodelet
        dynamic_reconfigure
        image_transport
        message_generation
        pcl_conversions
        pcl_ros
        sensor_msgs
        cv_bridge
        camera_info_manager
        pluginlib
        rm_msgs
        image_geometry
)

#find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        CATKIN_DEPENDS nodelet roscpp sensor_msgs dynamic_reconfigure pluginlib
)

include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/pcd_to_image.cpp
        include/pcd_to_image/img_proc.h)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES})

add_executable(img_proc src/img_proc.cpp)
target_link_libraries(img_proc
        ${catkin_LIBRARIES})

add_executable(pcd_proc src/pcd_proc.cpp)
target_link_libraries(pcd_proc
        ${catkin_LIBRARIES})

add_executable(down_sampling src/down_sampling.cpp)
target_link_libraries(down_sampling
        ${catkin_LIBRARIES})
