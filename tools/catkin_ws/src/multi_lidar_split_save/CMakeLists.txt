cmake_minimum_required(VERSION 2.8.12)
project(multi_lidar_split_save)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        pcl_ros
        pcl_conversions
        )

catkin_package(CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        geometry_msgs
        DEPENDS PCL
        )

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# MultiLidar split save four
add_executable(multi_lidar_split_save_four
        src/multi_lidar_split_save_four.cpp
        )
target_include_directories(multi_lidar_split_save_four PRIVATE
        include
        ${PCL_INCLUDE_DIRS}
)
target_link_libraries(multi_lidar_split_save_four
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)