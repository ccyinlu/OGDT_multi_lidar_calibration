% generate the pointcloud map from the absolute the trajectory

clc;
close all;

addpath('../../interface');
addpath('../../func');
addpath('../../thirdParty/yaml');

if ~exist('data_root')
    data_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync';
end

if ~exist('lidar_front_trajectory_absolute')
    lidar_front_trajectory_absolute = 'lidar_front_trajectory_absolute.txt';
end

if ~exist('lidar_left_trajectory_absolute')
    lidar_left_trajectory_absolute = 'lidar_left_trajectory_absolute.txt';
end

if ~exist('lidar_right_trajectory_absolute')
    lidar_right_trajectory_absolute = 'lidar_right_trajectory_absolute.txt';
end

if ~exist('lidar_back_trajectory_absolute')
    lidar_back_trajectory_absolute = 'lidar_back_trajectory_absolute.txt';
end

if ~exist('lidar_front_map_absolute')
    lidar_front_map_absolute = 'lidar_front_map_absolute.pcd';
end

if ~exist('lidar_left_map_absolute')
    lidar_left_map_absolute = 'lidar_left_map_absolute.pcd';
end

if ~exist('lidar_right_map_absolute')
    lidar_right_map_absolute = 'lidar_right_map_absolute.pcd';
end

if ~exist('lidar_back_map_absolute')
    lidar_back_map_absolute = 'lidar_back_map_absolute.pcd';
end

if ~exist('dir_lidar_front')
    dir_lidar_front = 'lidar_front';
end

if ~exist('dir_lidar_left')
    dir_lidar_left = 'lidar_left';
end

if ~exist('dir_lidar_right')
    dir_lidar_right = 'lidar_right';
end

if ~exist('dir_lidar_back')
    dir_lidar_back = 'lidar_back';
end

if ~exist('start_index')
    start_index = 1;
end

if ~exist('gridStep')
    gridStep = 0.1;
end

enables = {
    false, ... % #1 lidar_front pointcloud map generation from absolute trajectory
    false, ... % #2 lidar_left pointcloud map generation from absolute trajectory
    false, ... % #3 lidar_right pointcloud map generation from absolute trajectory
    true, ... % #4 lidar_back pointcloud map generation from absolute trajectory
}; 

lidar_front_data_dir = sprintf('%s/%s', data_root, dir_lidar_front);
lidar_front_observation_num = length(dir(fullfile(lidar_front_data_dir, '*.pcd')));
lidar_front_trajectory_absolute = dlmread(sprintf('%s/%s', data_root, lidar_front_trajectory_absolute), ',');

lidar_left_data_dir = sprintf('%s/%s', data_root, dir_lidar_left);
lidar_left_observation_num = length(dir(fullfile(lidar_left_data_dir, '*.pcd')));
lidar_left_trajectory_absolute = dlmread(sprintf('%s/%s', data_root, lidar_left_trajectory_absolute), ',');

lidar_right_data_dir = sprintf('%s/%s', data_root, dir_lidar_right);
lidar_right_observation_num = length(dir(fullfile(lidar_right_data_dir, '*.pcd')));
lidar_right_trajectory_absolute = dlmread(sprintf('%s/%s', data_root, lidar_right_trajectory_absolute), ',');

lidar_back_data_dir = sprintf('%s/%s', data_root, dir_lidar_back);
lidar_back_observation_num = length(dir(fullfile(lidar_back_data_dir, '*.pcd')));
lidar_back_trajectory_absolute = dlmread(sprintf('%s/%s', data_root, lidar_back_trajectory_absolute), ',');

%% #1
% lidar_front pointcloud map generation from absolute trajectory
if enables{1}
    fprintf('########################### #1 lidar_front pointcloud map generation from absolute trajectory   ##########################\n');
    for i = start_index : lidar_front_observation_num
        % read the pointcloud
        current_pc = pcread(sprintf('%s/%06d.pcd', lidar_front_data_dir, i));
        current_pose_absolute = lidar_front_trajectory_absolute(i, :);
        current_transform_matrix_absolute = transform_matrix_from_pose([
            current_pose_absolute(4) ...
            current_pose_absolute(5) ...
            current_pose_absolute(6) ...
            current_pose_absolute(1) ...
            current_pose_absolute(2) ...
            current_pose_absolute(3) ...
        ]);
        current_pc_transformed = pctransform(current_pc, affine3d(current_transform_matrix_absolute'));
        if i == start_index
            lidar_front_pc_map = current_pc_transformed;
        else
            lidar_front_pc_map = pcmerge(lidar_front_pc_map, current_pc_transformed, gridStep);
        end
        fprintf('processing frame %06d/%06d\n', i, lidar_front_observation_num);
    end
    pcwrite(lidar_front_pc_map, sprintf('%s/%s', data_root, lidar_front_map_absolute));
end

%% #2
% lidar_left pointcloud map generation from absolute trajectory
if enables{2}
    fprintf('########################### #2 lidar_left pointcloud map generation from absolute trajectory   ##########################\n');
    for i = start_index : lidar_left_observation_num
        % read the pointcloud
        current_pc = pcread(sprintf('%s/%06d.pcd', lidar_left_data_dir, i));
        current_pose_absolute = lidar_left_trajectory_absolute(i, :);
        current_transform_matrix_absolute = transform_matrix_from_pose([
            current_pose_absolute(4) ...
            current_pose_absolute(5) ...
            current_pose_absolute(6) ...
            current_pose_absolute(1) ...
            current_pose_absolute(2) ...
            current_pose_absolute(3) ...
        ]);
        current_pc_transformed = pctransform(current_pc, affine3d(current_transform_matrix_absolute'));
        if i == start_index
            lidar_left_pc_map = current_pc_transformed;
        else
            lidar_left_pc_map = pcmerge(lidar_left_pc_map, current_pc_transformed, gridStep);
        end
        fprintf('processing frame %06d/%06d\n', i, lidar_left_observation_num);
    end
    pcwrite(lidar_left_pc_map, sprintf('%s/%s', data_root, lidar_left_map_absolute));
end

%% #3
% lidar_right pointcloud map generation from absolute trajectory
if enables{3}
    fprintf('########################### #3 lidar_right pointcloud map generation from absolute trajectory   ##########################\n');
    for i = start_index : lidar_right_observation_num
        % read the pointcloud
        current_pc = pcread(sprintf('%s/%06d.pcd', lidar_right_data_dir, i));
        current_pose_absolute = lidar_right_trajectory_absolute(i, :);
        current_transform_matrix_absolute = transform_matrix_from_pose([
            current_pose_absolute(4) ...
            current_pose_absolute(5) ...
            current_pose_absolute(6) ...
            current_pose_absolute(1) ...
            current_pose_absolute(2) ...
            current_pose_absolute(3) ...
        ]);
        current_pc_transformed = pctransform(current_pc, affine3d(current_transform_matrix_absolute'));
        if i == start_index
            lidar_right_pc_map = current_pc_transformed;
        else
            lidar_right_pc_map = pcmerge(lidar_right_pc_map, current_pc_transformed, gridStep);
        end
        fprintf('processing frame %06d/%06d\n', i, lidar_right_observation_num);
    end
    pcwrite(lidar_right_pc_map, sprintf('%s/%s', data_root, lidar_right_map_absolute));
end

%% #4
% lidar_back pointcloud map generation from absolute trajectory
if enables{4}
    fprintf('########################### #4 lidar_back pointcloud map generation from absolute trajectory   ##########################\n');
    for i = start_index : lidar_back_observation_num
        % read the pointcloud
        current_pc = pcread(sprintf('%s/%06d.pcd', lidar_back_data_dir, i));
        current_pose_absolute = lidar_back_trajectory_absolute(i, :);
        current_transform_matrix_absolute = transform_matrix_from_pose([
            current_pose_absolute(4) ...
            current_pose_absolute(5) ...
            current_pose_absolute(6) ...
            current_pose_absolute(1) ...
            current_pose_absolute(2) ...
            current_pose_absolute(3) ...
        ]);
        current_pc_transformed = pctransform(current_pc, affine3d(current_transform_matrix_absolute'));
        if i == start_index
            lidar_back_pc_map = current_pc_transformed;
        else
            lidar_back_pc_map = pcmerge(lidar_back_pc_map, current_pc_transformed, gridStep);
        end
        fprintf('processing frame %06d/%06d\n', i, lidar_back_observation_num);
    end
    pcwrite(lidar_back_pc_map, sprintf('%s/%s', data_root, lidar_back_map_absolute));
end