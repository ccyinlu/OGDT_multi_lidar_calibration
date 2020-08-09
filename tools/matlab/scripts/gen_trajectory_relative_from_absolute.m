% get the trajectory relative from the absolute
clc;
close all;

addpath('../../interface');
addpath('../../func');
addpath('../../thirdParty/yaml');

if ~exist('data_root')
    data_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync/aloam';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~exist('file_lidar_front_trajectory_absolute')
    file_lidar_front_trajectory_absolute = 'lidar2_aloam_lidarPose_path.txt';
end

if ~exist('file_lidar_left_trajectory_absolute')
    file_lidar_left_trajectory_absolute = 'lidar1_aloam_lidarPose_path.txt';
end

if ~exist('file_lidar_right_trajectory_absolute')
    file_lidar_right_trajectory_absolute = 'lidar3_aloam_lidarPose_path.txt';
end

if ~exist('file_lidar_back_trajectory_absolute')
    file_lidar_back_trajectory_absolute = 'lidar4_aloam_lidarPose_path.txt';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~exist('file_lidar_front_trajectory_relative')
    file_lidar_front_trajectory_relative = 'lidar_front_trajectory_relative.txt';
end

if ~exist('file_lidar_left_trajectory_relative')
    file_lidar_left_trajectory_relative = 'lidar_left_trajectory_relative.txt';
end

if ~exist('file_lidar_right_trajectory_relative')
    file_lidar_right_trajectory_relative = 'lidar_right_trajectory_relative.txt';
end

if ~exist('file_lidar_back_trajectory_relative')
    file_lidar_back_trajectory_relative = 'lidar_back_trajectory_relative.txt';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~exist('sync_threshold')
    sync_threshold = 0.05;
end

enables = {
    true, % #1 generate relative pose from absolute from front lidar
    true, % #2 generate relative pose from absolute from left lidar
    true, % #3 generate relative pose from absolute from right lidar
    true, % #4 generate relative pose from absolute from back lidar
};

lidar_front_trajectory_relative_filename = sprintf('%s/%s', data_root, file_lidar_front_trajectory_relative);
lidar_left_trajectory_relative_filename = sprintf('%s/%s', data_root, file_lidar_left_trajectory_relative);
lidar_right_trajectory_relative_filename = sprintf('%s/%s', data_root, file_lidar_right_trajectory_relative);
lidar_back_trajectory_relative_filename = sprintf('%s/%s', data_root, file_lidar_back_trajectory_relative);

% [x y z qx qy qz qw stamp]
lidar_front_trajectory_absolute = dlmread(sprintf('%s/%s', data_root, file_lidar_front_trajectory_absolute), ',');
lidar_left_trajectory_absolute = dlmread(sprintf('%s/%s', data_root, file_lidar_left_trajectory_absolute), ',');
lidar_right_trajectory_absolute = dlmread(sprintf('%s/%s', data_root, file_lidar_right_trajectory_absolute), ',');
lidar_back_trajectory_absolute = dlmread(sprintf('%s/%s', data_root, file_lidar_back_trajectory_absolute), ',');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% #1 generate relative pose from absolute from front lidar
if enables{1}
    % get the observation number
    fprintf('processing the relative euler pose from the quat absolute pose\n');
    observation_num_lidar_front = size(lidar_front_trajectory_absolute, 1);
    lidar_front_trajectory_relative_euler_stamp = ones(observation_num_lidar_front, 7);
    for i = 1 : observation_num_lidar_front
        % get lidar_front_transform_matrix from quat pose
        current_lidar_front_transform_matrix_absolute = transform_matrix_from_pose_quat(lidar_front_trajectory_absolute(i, 1:7));
        if i == 1
            previous_lidar_front_transform_matrix_absolute = current_lidar_front_transform_matrix_absolute;
        end
        current_lidar_front_transform_matrix_relative = inv(previous_lidar_front_transform_matrix_absolute) * current_lidar_front_transform_matrix_absolute;
        current_lidar_front_pose_relative = pose_from_transform_matrix(current_lidar_front_transform_matrix_relative);

        lidar_front_trajectory_relative_euler_stamp(i, 1:6) = current_lidar_front_pose_relative;
        lidar_front_trajectory_relative_euler_stamp(i, 7) = lidar_front_trajectory_absolute(i, 8);

        previous_lidar_front_transform_matrix_absolute = current_lidar_front_transform_matrix_absolute;
        fprintf('processing lidar_front frame %06d/%06d\n', i, observation_num_lidar_front);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% #2 generate relative pose from absolute from left lidar
if enables{2}
    % get the observation number
    fprintf('processing the relative euler pose from the quat absolute pose\n');
    observation_num_lidar_left = size(lidar_left_trajectory_absolute, 1);
    lidar_left_trajectory_relative_euler_stamp = ones(observation_num_lidar_left, 7);
    for i = 1 : observation_num_lidar_left
        % get lidar_left_transform_matrix from quat pose
        current_lidar_left_transform_matrix_absolute = transform_matrix_from_pose_quat(lidar_left_trajectory_absolute(i, 1:7));
        if i == 1
            previous_lidar_left_transform_matrix_absolute = current_lidar_left_transform_matrix_absolute;
        end
        current_lidar_left_transform_matrix_relative = inv(previous_lidar_left_transform_matrix_absolute) * current_lidar_left_transform_matrix_absolute;
        current_lidar_left_pose_relative = pose_from_transform_matrix(current_lidar_left_transform_matrix_relative);

        lidar_left_trajectory_relative_euler_stamp(i, 1:6) = current_lidar_left_pose_relative;
        lidar_left_trajectory_relative_euler_stamp(i, 7) = lidar_left_trajectory_absolute(i, 8);

        previous_lidar_left_transform_matrix_absolute = current_lidar_left_transform_matrix_absolute;
        fprintf('processing lidar_left frame %06d/%06d\n', i, observation_num_lidar_left);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% #3 generate relative pose from absolute from right lidar
if enables{3}
    % get the observation number
    fprintf('processing the relative euler pose from the quat absolute pose\n');
    observation_num_lidar_right = size(lidar_right_trajectory_absolute, 1);
    lidar_right_trajectory_relative_euler_stamp = ones(observation_num_lidar_right, 7);
    for i = 1 : observation_num_lidar_right
        % get lidar_right_transform_matrix from quat pose
        current_lidar_right_transform_matrix_absolute = transform_matrix_from_pose_quat(lidar_right_trajectory_absolute(i, 1:7));
        if i == 1
            previous_lidar_right_transform_matrix_absolute = current_lidar_right_transform_matrix_absolute;
        end
        current_lidar_right_transform_matrix_relative = inv(previous_lidar_right_transform_matrix_absolute) * current_lidar_right_transform_matrix_absolute;
        current_lidar_right_pose_relative = pose_from_transform_matrix(current_lidar_right_transform_matrix_relative);

        lidar_right_trajectory_relative_euler_stamp(i, 1:6) = current_lidar_right_pose_relative;
        lidar_right_trajectory_relative_euler_stamp(i, 7) = lidar_right_trajectory_absolute(i, 8);

        previous_lidar_right_transform_matrix_absolute = current_lidar_right_transform_matrix_absolute;
        fprintf('processing lidar_right frame %06d/%06d\n', i, observation_num_lidar_right);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% #4 generate relative pose from absolute from back lidar
if enables{4}
    % get the observation number
    fprintf('processing the relative euler pose from the quat absolute pose\n');
    observation_num_lidar_back = size(lidar_back_trajectory_absolute, 1);
    lidar_back_trajectory_relative_euler_stamp = ones(observation_num_lidar_back, 7);
    for i = 1 : observation_num_lidar_back
        % get lidar_back_transform_matrix from quat pose
        current_lidar_back_transform_matrix_absolute = transform_matrix_from_pose_quat(lidar_back_trajectory_absolute(i, 1:7));
        if i == 1
            previous_lidar_back_transform_matrix_absolute = current_lidar_back_transform_matrix_absolute;
        end
        current_lidar_back_transform_matrix_relative = inv(previous_lidar_back_transform_matrix_absolute) * current_lidar_back_transform_matrix_absolute;
        current_lidar_back_pose_relative = pose_from_transform_matrix(current_lidar_back_transform_matrix_relative);

        lidar_back_trajectory_relative_euler_stamp(i, 1:6) = current_lidar_back_pose_relative;
        lidar_back_trajectory_relative_euler_stamp(i, 7) = lidar_back_trajectory_absolute(i, 8);

        previous_lidar_back_transform_matrix_absolute = current_lidar_back_transform_matrix_absolute;
        fprintf('processing lidar_back frame %06d/%06d\n', i, observation_num_lidar_back);
    end
end

%%
% sync the relative pose and save
fid_lidar_front_trajectory_relative = fopen(lidar_front_trajectory_relative_filename, 'w');
fid_lidar_left_trajectory_relative = fopen(lidar_left_trajectory_relative_filename, 'w');
fid_lidar_right_trajectory_relative = fopen(lidar_right_trajectory_relative_filename, 'w');
fid_lidar_back_trajectory_relative = fopen(lidar_back_trajectory_relative_filename, 'w');

for i = 1 : observation_num_lidar_front
    current_timestamp_baseline = lidar_front_trajectory_relative_euler_stamp(i, 7);

    % sync the timestamp
    [lidar_left_isFound, lidar_left_sync_index] = sync_time(current_timestamp_baseline, lidar_left_trajectory_relative_euler_stamp(:, 7), sync_threshold);
    [lidar_right_isFound, lidar_right_sync_index] = sync_time(current_timestamp_baseline, lidar_right_trajectory_relative_euler_stamp(:, 7), sync_threshold);
    [lidar_back_isFound, lidar_back_sync_index] = sync_time(current_timestamp_baseline, lidar_back_trajectory_relative_euler_stamp(:, 7), sync_threshold);

    if lidar_left_isFound && lidar_right_isFound && lidar_back_isFound
        % 
        current_lidar_front_pose_relative = lidar_front_trajectory_relative_euler_stamp(i, 1:6);
        current_lidar_left_pose_relative = lidar_left_trajectory_relative_euler_stamp(lidar_left_sync_index, 1:6);
        current_lidar_right_pose_relative = lidar_right_trajectory_relative_euler_stamp(lidar_right_sync_index, 1:6);
        current_lidar_back_pose_relative = lidar_back_trajectory_relative_euler_stamp(lidar_back_sync_index, 1:6);
        str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                    current_lidar_front_pose_relative(4), ...
                    current_lidar_front_pose_relative(5), ...
                    current_lidar_front_pose_relative(6), ...
                    current_lidar_front_pose_relative(1), ...
                    current_lidar_front_pose_relative(2), ...
                    current_lidar_front_pose_relative(3));
        fprintf(fid_lidar_front_trajectory_relative, str);
        str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                    current_lidar_left_pose_relative(4), ...
                    current_lidar_left_pose_relative(5), ...
                    current_lidar_left_pose_relative(6), ...
                    current_lidar_left_pose_relative(1), ...
                    current_lidar_left_pose_relative(2), ...
                    current_lidar_left_pose_relative(3));
        fprintf(fid_lidar_left_trajectory_relative, str);
        str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                    current_lidar_right_pose_relative(4), ...
                    current_lidar_right_pose_relative(5), ...
                    current_lidar_right_pose_relative(6), ...
                    current_lidar_right_pose_relative(1), ...
                    current_lidar_right_pose_relative(2), ...
                    current_lidar_right_pose_relative(3));
        fprintf(fid_lidar_right_trajectory_relative, str);
        str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                    current_lidar_back_pose_relative(4), ...
                    current_lidar_back_pose_relative(5), ...
                    current_lidar_back_pose_relative(6), ...
                    current_lidar_back_pose_relative(1), ...
                    current_lidar_back_pose_relative(2), ...
                    current_lidar_back_pose_relative(3));
        fprintf(fid_lidar_back_trajectory_relative, str);
    end

    fprintf('processing frame %06d/%06d\n', i, observation_num_lidar_front);
end

fclose(fid_lidar_front_trajectory_relative);
fclose(fid_lidar_left_trajectory_relative);
fclose(fid_lidar_right_trajectory_relative);
fclose(fid_lidar_back_trajectory_relative);

function [isFound, sync_index] = sync_time(current_timestamp, timestamps, stamp_threshold)
    delta_timestamp = abs(current_timestamp - timestamps);
    [min_delta_timestamp, index_delta_timestamp] = min(delta_timestamp);
    if min_delta_timestamp > stamp_threshold
        isFound = false;
        sync_index = -1;
    else
        isFound = true;
        sync_index = index_delta_timestamp;
    end
end
