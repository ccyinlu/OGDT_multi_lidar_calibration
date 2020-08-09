## prescan生成多激光雷达仿真数据

### 运行prescan_simulink_ros_multi_lidar_calibration
1. 编译  
`cd tools/catkin_ws`  
`catkin_make`  
`source devel/setup.bash`  
`roslaunch prescan_simulink_ros_multi_lidar_calibration demo_multi_lidar_data_save.launch`

### 运行prescan & simulink
`DemoSimuROS.m`  
运行simulink的模型文件  

值得注意的是，传感器的参数，尤其是水平角与垂直角，除了在Prescan中设置以外，在simulink里面也会有。具体内容在`pointCloudParam`中。

### 同步激光雷达文件
`cd tools/matlab/scripts/prescan`  
`gen_sync_data_from_timestamp.m`  

### 将同步文件载成ROSBAG  

#### 获得原始rosbag数据
获得rosbag，其中包括lidar1、lidar2、lidar3以及lidar4四个激光雷达topic  
第一个参数是rosbag包输出的路径，第二个数据是sync数据目录，第三个-f是第一帧的帧号，第四个-l是最后一帧帧号，默认rosbag包的名字是multi_lidar_calib.bag  
`cd tools/python/rosbag`  
`python multi_lidar_sync2bag.py /home/bingo/ethan/multi-lidar-calibration/Dataset/prescan /home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync -f 1 -l 275`  

#### 获得原始rosbag数据以及gnss还有拼接点云

首先获得各个激光雷达之间的相对位姿关系：  
`cd tools/matlab/scripts/prescan`  
`gen_transform_from_relative_pose.m`  

获得rosbag，其中包括gnss_pose、lidar1、lidar2、lidar3、lidar4以及根据真值拼接出来的点云  
第一个参数是rosbag包输出的路径，第二个数据是sync数据目录，第三个-f是第一帧的帧号，第四个-l是最后一帧帧号，默认rosbag包的名字是multi_lidar_calib.bag  
`cd tools/python/rosbag`  
`python multi_lidar_full_sync2bag.py /home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/test /home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync -f 1 -l 275`  