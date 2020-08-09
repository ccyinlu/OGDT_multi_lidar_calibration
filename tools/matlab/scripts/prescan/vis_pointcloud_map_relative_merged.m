% generate the pointcloud map from the relative the trajectory

clc;
close all;

addpath('../../interface');
addpath('../../func');
addpath('../../thirdParty/yaml');

if ~exist('data_root')
    data_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync';
end

if ~exist('lidar_front_map_relative')
    lidar_front_map_relative = 'lidar_front_map_relative.pcd';
end

if ~exist('lidar_left_map_relative')
    lidar_left_map_relative = 'lidar_left_map_relative.pcd';
end

if ~exist('lidar_right_map_relative')
    lidar_right_map_relative = 'lidar_right_map_relative.pcd';
end

if ~exist('lidar_back_map_relative')
    lidar_back_map_relative = 'lidar_back_map_relative.pcd';
end

if ~exist('gridStep')
    gridStep = 0.1;
end

% load the relative transform gt
lidar_left_to_front_pose = loadPoseYaml(sprintf('%s/lidar_1_to_2_gt_6dof.txt', data_root));
lidar_right_to_front_pose = loadPoseYaml(sprintf('%s/lidar_3_to_2_gt_6dof.txt', data_root));
lidar_back_to_front_pose = loadPoseYaml(sprintf('%s/lidar_4_to_2_gt_6dof.txt', data_root));

transform_matrix_left_to_front = transform_matrix_from_pose(lidar_left_to_front_pose);
transform_matrix_right_to_front = transform_matrix_from_pose(lidar_right_to_front_pose);
transform_matrix_back_to_front = transform_matrix_from_pose(lidar_back_to_front_pose);

% load the pc_map relative
pc_lidar_front_map_relative = pcread(sprintf('%s/%s', data_root, lidar_front_map_relative));
pc_lidar_left_map_relative = pcread(sprintf('%s/%s', data_root, lidar_left_map_relative));
pc_lidar_right_map_relative = pcread(sprintf('%s/%s', data_root, lidar_right_map_relative));
pc_lidar_back_map_relative = pcread(sprintf('%s/%s', data_root, lidar_back_map_relative));

points_lidar_front_map = pc_lidar_front_map_relative.Location;
points_lidar_left_map = pc_lidar_left_map_relative.Location;
points_lidar_right_map = pc_lidar_right_map_relative.Location;
points_lidar_back_map = pc_lidar_back_map_relative.Location;

% left-multiply
points_lidar_left_map_left_transformed = (transform_matrix_left_to_front * [points_lidar_left_map ones(size(points_lidar_left_map, 1), 1)]')';
points_lidar_right_map_left_transformed = (transform_matrix_right_to_front * [points_lidar_right_map ones(size(points_lidar_right_map, 1), 1)]')';
points_lidar_back_map_left_transformed = (transform_matrix_back_to_front * [points_lidar_back_map ones(size(points_lidar_back_map, 1), 1)]')';

% figure();
% scatter3(points_lidar_front_map(:, 1), points_lidar_front_map(:, 2), points_lidar_front_map(:, 3), 5, [1 0 0], 'filled');
% hold on;
% scatter3(points_lidar_left_map_left_transformed(:, 1), points_lidar_left_map_left_transformed(:, 2), points_lidar_left_map_left_transformed(:, 3), 5, [0 1 0], 'filled');
% hold on;
% scatter3(points_lidar_right_map_left_transformed(:, 1), points_lidar_right_map_left_transformed(:, 2), points_lidar_right_map_left_transformed(:, 3), 5, [0 0 1], 'filled');
% hold on;
% scatter3(points_lidar_back_map_left_transformed(:, 1), points_lidar_back_map_left_transformed(:, 2), points_lidar_back_map_left_transformed(:, 3), 5, [0 0 0], 'filled');
% axis equal;

pc_lidar_front_map = pointCloud(points_lidar_front_map);
pc_lidar_left_map_left_transformed = pointCloud(points_lidar_left_map_left_transformed(:, 1:3));
pc_lidar_right_map_left_transformed = pointCloud(points_lidar_right_map_left_transformed(:, 1:3));
pc_lidar_back_map_left_transformed = pointCloud(points_lidar_back_map_left_transformed(:, 1:3));

lidar_front_pc_map_merged = pc_lidar_front_map;
lidar_front_pc_map_merged = pcmerge(lidar_front_pc_map_merged, pc_lidar_left_map_left_transformed, gridStep);
lidar_front_pc_map_merged = pcmerge(lidar_front_pc_map_merged, pc_lidar_right_map_left_transformed, gridStep);
lidar_front_pc_map_merged = pcmerge(lidar_front_pc_map_merged, pc_lidar_back_map_left_transformed, gridStep);

figure();
pcshow(lidar_front_pc_map_merged);
