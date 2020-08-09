% gen the calib_before leveling pose guess

clc;
close all;

if ~exist('data_root')
  data_root = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_sync';
end

if ~exist('tools_matlab')
  tools_matlab = '/home/bingo/ethan/multi-lidar-calibration/tools/matlab';
end

addpath([tools_matlab '/' 'interface']);
addpath([tools_matlab '/' 'thirdParty/yaml']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~exist('calib_gt_path')
  calib_gt_path = 'calib_gt';
end

if ~exist('calib_gt_leveling_path')
  calib_gt_leveling_path = 'calib_gt_leveling';
end

if ~exist('lidar_front_pose_gt_filename')
  lidar_front_pose_gt_filename = 'lidar_front_pose_relative.txt';
end

if ~exist('lidar_left_pose_gt_filename')
  lidar_left_pose_gt_filename = 'lidar_left_pose_relative.txt';
end

if ~exist('lidar_right_pose_gt_filename')
  lidar_right_pose_gt_filename = 'lidar_right_pose_relative.txt';
end

if ~exist('lidar_back_pose_gt_filename')
  lidar_back_pose_gt_filename = 'lidar_back_pose_relative.txt';
end

% create the calib_gt leveing path
calib_gt_leveling_dir = sprintf('%s/%s', data_root, calib_gt_leveling_path);
if ~exist(calib_gt_leveling_dir)
  command = sprintf('mkdir -p %s', calib_gt_leveling_dir);
  system(command);
end

% load the calib_gt pose and gen the calib_gt_leveling pose
% [roll pitch yaw x y z] [rad rad rad meters meters meters]
lidar_left_to_front_pose_gt = loadPoseYaml(sprintf('%s/%s/lidar_1_to_2_gt_6dof.txt', data_root, calib_gt_path));
lidar_right_to_front_pose_gt = loadPoseYaml(sprintf('%s/%s/lidar_3_to_2_gt_6dof.txt', data_root, calib_gt_path));
lidar_back_to_front_pose_gt = loadPoseYaml(sprintf('%s/%s/lidar_4_to_2_gt_6dof.txt', data_root, calib_gt_path));

% load the gt pose related to front
% [roll pitch yaw x y z] [rad rad rad meters meters meters]
lidar_front_pose_gt = loadPoseYaml(sprintf('%s/%s', data_root, lidar_front_pose_gt_filename));
lidar_left_pose_gt = loadPoseYaml(sprintf('%s/%s', data_root, lidar_left_pose_gt_filename));
lidar_right_pose_gt = loadPoseYaml(sprintf('%s/%s', data_root, lidar_right_pose_gt_filename));
lidar_back_pose_gt = loadPoseYaml(sprintf('%s/%s', data_root, lidar_back_pose_gt_filename));

% [roll pitch yaw x y z]
% [roll pitch z] refers the roll, pitch and z offset to the ground
% [x y yaw] refers to the left, right and back refers to the front lidar
gt_6dof_front_to_ground_front_filename = sprintf('%s/lidar_2_estimated_6dof.txt', calib_gt_leveling_dir);
outputPoseYaml([lidar_front_pose_gt(1); ...
                lidar_front_pose_gt(2); ...
                0; ...
                0; ...
                0; ...
                lidar_front_pose_gt(6)], ...
                gt_6dof_front_to_ground_front_filename);

gt_6dof_left_to_ground_front_filename = sprintf('%s/lidar_1_estimated_6dof.txt', calib_gt_leveling_dir);
outputPoseYaml([lidar_left_pose_gt(1); ...
                lidar_left_pose_gt(2); ...
                lidar_left_to_front_pose_gt(3); ...
                lidar_left_to_front_pose_gt(4); ...
                lidar_left_to_front_pose_gt(5); ...
                lidar_left_pose_gt(6)], ...
                gt_6dof_left_to_ground_front_filename);

gt_6dof_right_to_ground_front_filename = sprintf('%s/lidar_3_estimated_6dof.txt', calib_gt_leveling_dir);
outputPoseYaml([lidar_right_pose_gt(1); ...
                lidar_right_pose_gt(2); ...
                lidar_right_to_front_pose_gt(3); ...
                lidar_right_to_front_pose_gt(4); ...
                lidar_right_to_front_pose_gt(5); ...
                lidar_right_pose_gt(6)], ...
                gt_6dof_right_to_ground_front_filename);

gt_6dof_back_to_ground_front_filename = sprintf('%s/lidar_4_estimated_6dof.txt', calib_gt_leveling_dir);
outputPoseYaml([lidar_back_pose_gt(1); ...
                lidar_back_pose_gt(2); ...
                lidar_back_to_front_pose_gt(3); ...
                lidar_back_to_front_pose_gt(4); ...
                lidar_back_to_front_pose_gt(5); ...
                lidar_back_pose_gt(6)], ...
                gt_6dof_back_to_ground_front_filename);
