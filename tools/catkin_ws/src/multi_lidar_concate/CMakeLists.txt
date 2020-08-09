cmake_minimum_required(VERSION 2.8.12)
project(multi_lidar_concate)

set(yaml-cpp_DIR /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp)

IF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
ENDIF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        pcl_ros
        pcl_conversions
        yaml-cpp
        )

catkin_package(CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        geometry_msgs
        DEPENDS PCL
        )

MESSAGE( STATUS "yaml-cpp_INCLUDE_DIR:" ${YAML_CPP_INCLUDE_DIR})
MESSAGE( STATUS "yaml-cpp_LIBRARIES:" ${YAML_CPP_LIBRARIES})
###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        ${YAML_CPP_INCLUDE_DIR}
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# MultiLidar concate two
add_executable(multi_lidar_concate_two
        src/multi_lidar_concate_two.cpp
        )
target_include_directories(multi_lidar_concate_two PRIVATE
        include
        ${PCL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR})
        
target_link_libraries(multi_lidar_concate_two
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES})

# # MultiLidar concate three
# add_executable(multi_lidar_concate_three
#         src/multi_lidar_concate_three.cpp
#         )
# target_include_directories(multi_lidar_concate_three PRIVATE
#         include
#         ${PCL_INCLUDE_DIRS})
# target_link_libraries(multi_lidar_concate_three
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES})

# MultiLidar concate four
add_executable(multi_lidar_concate_four
        src/multi_lidar_concate_four.cpp
        )
target_include_directories(multi_lidar_concate_four PRIVATE
        include
        ${PCL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(multi_lidar_concate_four
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES})