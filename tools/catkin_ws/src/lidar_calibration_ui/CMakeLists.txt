cmake_minimum_required(VERSION 2.8.3)
project(multi_lidar_concate_ui)

# set yaml path by current system processor
MESSAGE(STATUS "current platform: ${CMAKE_HOST_SYSTEM_PROCESSOR} ")
IF (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(yaml-cpp_DIR /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp)
ELSEIF (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(yaml-cpp_DIR /usr/lib/aarch64-linux-gnu/cmake/yaml-cpp)
ENDIF (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")

IF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
ENDIF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(Qt5Widgets REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        pcl_conversions
        yaml-cpp
        )

catkin_package(
        )

MESSAGE( STATUS "yaml-cpp_INCLUDE_DIR:" ${YAML_CPP_INCLUDE_DIR})
MESSAGE( STATUS "yaml-cpp_LIBRARIES:" ${YAML_CPP_LIBRARIES})

###########
## Build ##
###########

## QT必须 ##
set(CMAKE_AUTOMOC ON)

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        ${YAML_CPP_INCLUDE_DIR}
        ${Qt5Widgets_INCLUDE_DIRS}
        src/ui
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# MultiLidar concate four
add_executable(${PROJECT_NAME}_node
        src/multi_lidar_concate_ui_node.cpp
        src/mainwindow.cpp
)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${Qt5Widgets_LIBRARIES})