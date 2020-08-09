% vis delta pose relative

clc;
close all;
clear;

addpath('../../interface');
addpath('../../func');
addpath('../../thirdParty/yaml');

if ~exist('data_root')
    data_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync';
end

if ~exist('lidar_front_trajectory_relative')
    lidar_front_trajectory_relative = 'lidar_front_trajectory_relative.txt';
end

if ~exist('lidar_left_trajectory_relative')
    lidar_left_trajectory_relative = 'lidar_left_trajectory_relative.txt';
end

if ~exist('lidar_right_trajectory_relative')
    lidar_right_trajectory_relative = 'lidar_right_trajectory_relative.txt';
end

if ~exist('lidar_back_trajectory_relative')
    lidar_back_trajectory_relative = 'lidar_back_trajectory_relative.txt';
end

% load the relative transform gt
lidar_left_to_front_pose = loadPoseYaml(sprintf('%s/lidar_1_to_2_gt_6dof.txt', data_root));
lidar_right_to_front_pose = loadPoseYaml(sprintf('%s/lidar_3_to_2_gt_6dof.txt', data_root));
lidar_back_to_front_pose = loadPoseYaml(sprintf('%s/lidar_4_to_2_gt_6dof.txt', data_root));

transform_matrix_left_to_front = transform_matrix_from_pose(lidar_left_to_front_pose);
transform_matrix_right_to_front = transform_matrix_from_pose(lidar_right_to_front_pose);
transform_matrix_back_to_front = transform_matrix_from_pose(lidar_back_to_front_pose);

lidar_front_trajectory_relative = dlmread(sprintf('%s/%s', data_root, lidar_front_trajectory_relative), ',');
lidar_left_trajectory_relative = dlmread(sprintf('%s/%s', data_root, lidar_left_trajectory_relative), ',');
lidar_right_trajectory_relative = dlmread(sprintf('%s/%s', data_root, lidar_right_trajectory_relative), ',');
lidar_back_trajectory_relative = dlmread(sprintf('%s/%s', data_root, lidar_back_trajectory_relative), ',');

lidar_left_trajectory_relative_estimated = zeros(size(lidar_left_trajectory_relative, 1), 6);
lidar_right_trajectory_relative_estimated = zeros(size(lidar_right_trajectory_relative, 1), 6);
lidar_back_trajectory_relative_estimated = zeros(size(lidar_back_trajectory_relative, 1), 6);

for i = 1 : size(lidar_front_trajectory_relative, 1)
    current_front_pose_relative = lidar_front_trajectory_relative(i, :);
    current_front_transform_matrix_relative = transform_matrix_from_pose([
        current_front_pose_relative(4) ...
        current_front_pose_relative(5) ...
        current_front_pose_relative(6) ...
        current_front_pose_relative(1) ...
        current_front_pose_relative(2) ...
        current_front_pose_relative(3) ...
    ]);
    lidar_left_matrix_relative_estimated = inv(transform_matrix_left_to_front) * current_front_transform_matrix_relative * transform_matrix_left_to_front;
    lidar_left_pose_relative_estimated = pose_from_transform_matrix(lidar_left_matrix_relative_estimated);
    lidar_left_trajectory_relative_estimated(i, :) = [
        lidar_left_pose_relative_estimated(4) ...
        lidar_left_pose_relative_estimated(5) ...
        lidar_left_pose_relative_estimated(6) ...
        lidar_left_pose_relative_estimated(1) ...
        lidar_left_pose_relative_estimated(2) ...
        lidar_left_pose_relative_estimated(3) ...
    ];
    lidar_right_matrix_relative_estimated = inv(transform_matrix_right_to_front) * current_front_transform_matrix_relative * transform_matrix_right_to_front;
    lidar_right_pose_relative_estimated = pose_from_transform_matrix(lidar_right_matrix_relative_estimated);
    lidar_right_trajectory_relative_estimated(i, :) = [
        lidar_right_pose_relative_estimated(4) ...
        lidar_right_pose_relative_estimated(5) ...
        lidar_right_pose_relative_estimated(6) ...
        lidar_right_pose_relative_estimated(1) ...
        lidar_right_pose_relative_estimated(2) ...
        lidar_right_pose_relative_estimated(3) ...
    ];
    lidar_back_matrix_relative_estimated = inv(transform_matrix_back_to_front) * current_front_transform_matrix_relative * transform_matrix_back_to_front;
    lidar_back_pose_relative_estimated = pose_from_transform_matrix(lidar_back_matrix_relative_estimated);
    lidar_back_trajectory_relative_estimated(i, :) = [
        lidar_back_pose_relative_estimated(4) ...
        lidar_back_pose_relative_estimated(5) ...
        lidar_back_pose_relative_estimated(6) ...
        lidar_back_pose_relative_estimated(1) ...
        lidar_back_pose_relative_estimated(2) ...
        lidar_back_pose_relative_estimated(3) ...
    ];

    fprintf('processing %06d/%06d\n', i, size(lidar_front_trajectory_relative, 1));
end

delta_left_to_front_xyz = lidar_left_trajectory_relative(:, 1:3) - lidar_left_trajectory_relative_estimated(:, 1:3);
delta_left_to_front_rpy = lidar_left_trajectory_relative(:, 4:5) - lidar_left_trajectory_relative_estimated(:, 4:5);

delta_right_to_front_xyz = lidar_right_trajectory_relative(:, 1:3) - lidar_right_trajectory_relative_estimated(:, 1:3);
delta_right_to_front_rpy = lidar_right_trajectory_relative(:, 4:5) - lidar_right_trajectory_relative_estimated(:, 4:5);

delta_back_to_front_xyz = lidar_back_trajectory_relative(:, 1:3) - lidar_back_trajectory_relative_estimated(:, 1:3);
delta_back_to_front_rpy = lidar_back_trajectory_relative(:, 4:5) - lidar_back_trajectory_relative_estimated(:, 4:5);

figure();
plot(sum(abs(delta_left_to_front_xyz).^2,2).^(1/2));
figure();
plot(sum(abs(delta_left_to_front_rpy).^2,2).^(1/2));

figure();
plot(sum(abs(delta_right_to_front_xyz).^2,2).^(1/2));
figure();
plot(sum(abs(delta_right_to_front_rpy).^2,2).^(1/2));

figure();
plot(sum(abs(delta_back_to_front_xyz).^2,2).^(1/2));
figure();
plot(sum(abs(delta_back_to_front_rpy).^2,2).^(1/2));
