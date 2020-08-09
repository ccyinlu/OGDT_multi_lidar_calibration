% test linefit segmentation
close all;
clc;

if ~exist('data_root')
  data_root = '/media/bingo/SSD/multi_lidar_calib_data/sharingVAN/20200706/lidar_calibration';
end

if ~exist('rosbag_filename')
  rosbag_filename = 'source.bag';
end

if ~exist('start_index')
  start_index = 1;
end

if ~exist('lidar_front_dir')
  lidar_front_dir = lidar_front;
end

if ~exist('lidar_left_dir')
  lidar_left_dir = lidar_left;
end

if ~exist('lidar_right_dir')
  lidar_right_dir = lidar_right;
end

if ~exist('lidar_back_dir')
  lidar_back_dir = lidar_back;
end

if ~exist('lidar_front_topic')
  lidar_front_topic = '/lidar2/points_raw';
end

if ~exist('lidar_left_topic')
  lidar_left_topic = '/lidar1/points_raw';
end

if ~exist('lidar_right_topic')
  lidar_right_topic = '/lidar3/points_raw';
end

if ~exist('lidar_back_topic')
  lidar_back_topic = '/lidar4/points_raw';
end

lidar_front_dir_full = sprintf('%s/%s', data_root, lidar_front_dir);
lidar_left_dir_full = sprintf('%s/%s', data_root, lidar_left_dir);
lidar_right_dir_full = sprintf('%s/%s', data_root, lidar_right_dir);
lidar_back_dir_full = sprintf('%s/%s', data_root, lidar_back_dir);

if ~exist(lidar_front_dir_full)
  command = sprintf('mkdir -p %s', lidar_front_dir_full);
  system(command);
end

if ~exist(lidar_left_dir_full)
  command = sprintf('mkdir -p %s', lidar_left_dir_full);
  system(command);
end

if ~exist(lidar_right_dir_full)
  command = sprintf('mkdir -p %s', lidar_right_dir_full);
  system(command);
end

if ~exist(lidar_back_dir_full)
  command = sprintf('mkdir -p %s', lidar_back_dir_full);
  system(command);
end

% load the rosbag
rosbag_filename_full = sprintf('%s/%s', data_root, rosbag_filename);
cur_bag = rosbag(rosbag_filename_full);

lidar_front_rosbag = select(cur_bag, 'Topic', lidar_front_topic);
lidar_front_Num = lidar_front_rosbag.NumMessages;