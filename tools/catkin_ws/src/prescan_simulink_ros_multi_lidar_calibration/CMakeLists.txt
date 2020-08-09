cmake_minimum_required(VERSION 2.8.3)
project(prescan_simulink_ros_multi_lidar_calibration)

find_package(catkin REQUIRED)

#catkin_metapackage()
catkin_package()

execute_process(COMMAND chmod +x "${CMAKE_CURRENT_SOURCE_DIR}/scripts/multi_lidar_data_save.py")

catkin_install_python(PROGRAMS
  scripts/multi_lidar_data_save.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/demo_multi_lidar_data_save.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
