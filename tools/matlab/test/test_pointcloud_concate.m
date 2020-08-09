% test pointcloud concate

clc;
close all;

addpath('../interface');
addpath('../func');
addpath('../thirdParty/yaml');

if ~exist('data_root')
    data_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync';
end

if ~exist('save_root')
    save_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync';
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

file_lidar_left_to_front_gt_filename = [save_root '/' file_lidar_left_to_front_gt];
file_lidar_right_to_front_gt_filename = [save_root '/' file_lidar_right_to_front_gt];
file_lidar_back_to_front_gt_filename = [save_root '/' file_lidar_back_to_front_gt];

lidar_left_to_front_pose = loadPoseYaml(file_lidar_left_to_front_gt_filename);
lidar_right_to_front_pose = loadPoseYaml(file_lidar_right_to_front_gt_filename);
lidar_back_to_front_pose = loadPoseYaml(file_lidar_back_to_front_gt_filename);

transform_matrix_left_to_front = transform_matrix_from_pose(lidar_left_to_front_pose);
transform_matrix_right_to_front = transform_matrix_from_pose(lidar_right_to_front_pose);
transform_matrix_back_to_front = transform_matrix_from_pose(lidar_back_to_front_pose);

% test data
pointcloud_id = 30;

pc_lidar_front_test = pcread(sprintf('%s/lidar_front/%06d.pcd', data_root, pointcloud_id));
pc_lidar_left_test = pcread(sprintf('%s/lidar_left/%06d.pcd', data_root, pointcloud_id));
pc_lidar_right_test = pcread(sprintf('%s/lidar_right/%06d.pcd', data_root, pointcloud_id));
pc_lidar_back_test = pcread(sprintf('%s/lidar_back/%06d.pcd', data_root, pointcloud_id));

points_lidar_front_test = pc_lidar_front_test.Location;
points_lidar_left_test = pc_lidar_left_test.Location;
points_lidar_right_test = pc_lidar_right_test.Location;
points_lidar_back_test = pc_lidar_back_test.Location;

% left-multiply
points_lidar_left_test_left_transformed = (transform_matrix_left_to_front * [points_lidar_left_test ones(size(points_lidar_left_test, 1), 1)]')';
points_lidar_right_test_left_transformed = (transform_matrix_right_to_front * [points_lidar_right_test ones(size(points_lidar_right_test, 1), 1)]')';
points_lidar_back_test_left_transformed = (transform_matrix_back_to_front * [points_lidar_back_test ones(size(points_lidar_back_test, 1), 1)]')';

% right-multiply
points_lidar_left_test_right_transformed = ([points_lidar_left_test ones(size(points_lidar_left_test, 1), 1)] * transform_matrix_left_to_front);
points_lidar_right_test_right_transformed = ([points_lidar_right_test ones(size(points_lidar_right_test, 1), 1)] * transform_matrix_right_to_front);
points_lidar_back_test_right_transformed = ([points_lidar_back_test ones(size(points_lidar_back_test, 1), 1)] * transform_matrix_back_to_front);

figure();
scatter3(points_lidar_front_test(:, 1), points_lidar_front_test(:, 2), points_lidar_front_test(:, 3), 5, [1 0 0], 'filled');
hold on;
scatter3(points_lidar_left_test_left_transformed(:, 1), points_lidar_left_test_left_transformed(:, 2), points_lidar_left_test_left_transformed(:, 3), 5, [0 1 0], 'filled');
hold on;
scatter3(points_lidar_right_test_left_transformed(:, 1), points_lidar_right_test_left_transformed(:, 2), points_lidar_right_test_left_transformed(:, 3), 5, [0 0 1], 'filled');
hold on;
scatter3(points_lidar_back_test_left_transformed(:, 1), points_lidar_back_test_left_transformed(:, 2), points_lidar_back_test_left_transformed(:, 3), 5, [0 0 0], 'filled');
axis equal;

figure();
scatter3(points_lidar_front_test(:, 1), points_lidar_front_test(:, 2), points_lidar_front_test(:, 3), 5, [1 0 0], 'filled');
hold on;
scatter3(points_lidar_left_test_right_transformed(:, 1), points_lidar_left_test_right_transformed(:, 2), points_lidar_left_test_right_transformed(:, 3), 5, [0 1 0], 'filled');
hold on;
scatter3(points_lidar_right_test_right_transformed(:, 1), points_lidar_right_test_right_transformed(:, 2), points_lidar_right_test_right_transformed(:, 3), 5, [0 0 1], 'filled');
hold on;
scatter3(points_lidar_back_test_right_transformed(:, 1), points_lidar_back_test_right_transformed(:, 2), points_lidar_back_test_right_transformed(:, 3), 5, [0 0 0], 'filled');
axis equal;