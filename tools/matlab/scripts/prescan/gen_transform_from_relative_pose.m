% gen transformation between lidars from ground truth relative pose
% 从传感器相对于车辆的位姿信息中估计激光雷达之间的相对位姿

clc;
close all;

addpath('../../interface');
addpath('../../func');
addpath('../../thirdParty/yaml');

if ~exist('data_root')
    % data_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync';
    data_root = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_sync';
end

if ~exist('save_root')
    % save_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync';
    save_root = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_sync';
end

if ~exist('file_lidar_front_pose_relative')
    file_lidar_front_pose_relative = 'lidar_front_pose_relative.txt';
end

if ~exist('file_lidar_left_pose_relative')
    file_lidar_left_pose_relative = 'lidar_left_pose_relative.txt';
end

if ~exist('file_lidar_right_pose_relative')
    file_lidar_right_pose_relative = 'lidar_right_pose_relative.txt';
end

if ~exist('file_lidar_back_pose_relative')
    file_lidar_back_pose_relative = 'lidar_back_pose_relative.txt';
end

if ~exist('file_lidar_left_to_front_gt')
    file_lidar_left_to_front_gt = 'lidar_1_to_2_gt_6dof.txt';
end

if ~exist('file_lidar_right_to_front_gt')
    file_lidar_right_to_front_gt = 'lidar_3_to_2_gt_6dof.txt';
end

if ~exist('file_lidar_back_to_front_gt')
    file_lidar_back_to_front_gt = 'lidar_4_to_2_gt_6dof.txt';
end

intrinsic_matrix = eye(4);
intrinsic_rot = eul2rotm([0 0 0]);
intrinsic_matrix(1:3, 1:3) = intrinsic_rot;

% load the pose_relative
lidar_front_pose_to_vehicle = loadPoseYaml([data_root '/' file_lidar_front_pose_relative]);
lidar_left_pose_to_vehicle = loadPoseYaml([data_root '/' file_lidar_left_pose_relative]);
lidar_right_pose_to_vehicle = loadPoseYaml([data_root '/' file_lidar_right_pose_relative]);
lidar_back_pose_to_vehicle = loadPoseYaml([data_root '/' file_lidar_back_pose_relative]);

transform_matrix_front_to_vehicle = transform_matrix_from_pose(lidar_front_pose_to_vehicle);
transform_matrix_left_to_vehicle = transform_matrix_from_pose(lidar_left_pose_to_vehicle);
transform_matrix_right_to_vehicle = transform_matrix_from_pose(lidar_right_pose_to_vehicle);
transform_matrix_back_to_vehicle = transform_matrix_from_pose(lidar_back_pose_to_vehicle);

transform_matrix_front_to_vehicle = transform_matrix_front_to_vehicle * intrinsic_matrix;
transform_matrix_left_to_vehicle = transform_matrix_left_to_vehicle * intrinsic_matrix;
transform_matrix_right_to_vehicle = transform_matrix_right_to_vehicle * intrinsic_matrix;
transform_matrix_back_to_vehicle = transform_matrix_back_to_vehicle * intrinsic_matrix;

% left_to_vehicle = front_to_vehicle * left_to_front
left_to_front_transform = inv(transform_matrix_front_to_vehicle) * transform_matrix_left_to_vehicle;
% right_to_vehicle = front_to_vehicle * right_to_front
right_to_front_transform = inv(transform_matrix_front_to_vehicle) * transform_matrix_right_to_vehicle;
% back_to_vehicle = front_to_vehicle * back_to_front
back_to_front_transform = inv(transform_matrix_front_to_vehicle) * transform_matrix_back_to_vehicle;

% save the relative transformation
left_to_front_pose = pose_from_transform_matrix(left_to_front_transform);
right_to_front_pose = pose_from_transform_matrix(right_to_front_transform);
back_to_front_pose = pose_from_transform_matrix(back_to_front_transform);

outputPoseYaml(left_to_front_pose, [save_root '/' file_lidar_left_to_front_gt]);
outputPoseYaml(right_to_front_pose, [save_root '/' file_lidar_right_to_front_gt]);
outputPoseYaml(back_to_front_pose, [save_root '/' file_lidar_back_to_front_gt]);
fprintf('done!\n');