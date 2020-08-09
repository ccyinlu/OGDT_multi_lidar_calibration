% extract the pointcloud file from the rosbag

clc;
close all;

addpath('../utils/interface');

rosbag_filename = '/media/bingo/CCYINLU/DATA/sharingVAN/sharing-van-1.0plus/2#/2020-04-30/lidar_calib.bag';
pointcloud_topic = '/lidar1/points_raw';

extract_id = 1;

pointcloud_filename = sprintf('%s/lidar1_%06d.pcd','/media/bingo/CCYINLU/DATA/sharingVAN/sharing-van-1.0plus/2#/2020-04-30', extract_id);

% load the rosbag

bag_ = rosbag(rosbag_filename);
lidar1_rosbag_selected = select(bag_, 'Topic', pointcloud_topic);

% read the messages
messages_ = readMessages(bag_, 1);
point_xyz = readXYZ(messages_{1});
point_intensity = readIntensity(messages_{1});
ptCloud = pointCloud(point_xyz, 'Intensity', single(point_intensity));
pcwrite(ptCloud, pointcloud_filename, 'Encoding', 'binary');

fprintf('done\n');