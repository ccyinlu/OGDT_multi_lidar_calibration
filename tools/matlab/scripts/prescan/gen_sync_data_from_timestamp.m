% get the sync file from the timestamp file

clc;
close all;

if ~exist('data_root_timestamp')
    % data_root_timestamp = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt';
    data_root_timestamp = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_raw';
end

if ~exist('data_root')
    % data_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync';
    data_root = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_sync';
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

if ~exist('dir_vehicle_pose')
    dir_vehicle_pose = 'vehicle_pose';
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

if ~exist('start_index')
    start_index = 1;
end

if ~exist('sync_threshold')
    sync_threshold = 0.05;
end

enables = {
    true, ... % #1 prepare the pose relative txt file
    true, ... % #2 sync the lidar point cloud cooresponding to the vehicle pose
};

%% #1
% prepare the pose relative txt file
if enables{1}
    fprintf('########################### #1 prepare the pose relative txt file   ##########################\n');
    if exist(data_root, 'dir') ~= 7
        fprintf('data_root directory not exist, it will be created!\n');
        mkdir(data_root);
    end

    file_lidar_front_pose_relative_timestamp = sprintf('%s/%s', data_root_timestamp, file_lidar_front_pose_relative);
    file_lidar_front_pose_relative_sync = sprintf('%s/%s', data_root, file_lidar_front_pose_relative);
    copyfile(file_lidar_front_pose_relative_timestamp, file_lidar_front_pose_relative_sync);

    file_lidar_left_pose_relative_timestamp = sprintf('%s/%s', data_root_timestamp, file_lidar_left_pose_relative);
    file_lidar_left_pose_relative_sync = sprintf('%s/%s', data_root, file_lidar_left_pose_relative);
    copyfile(file_lidar_left_pose_relative_timestamp, file_lidar_left_pose_relative_sync);

    file_lidar_right_pose_relative_timestamp = sprintf('%s/%s', data_root_timestamp, file_lidar_right_pose_relative);
    file_lidar_right_pose_relative_sync = sprintf('%s/%s', data_root, file_lidar_right_pose_relative);
    copyfile(file_lidar_right_pose_relative_timestamp, file_lidar_right_pose_relative_sync);

    file_lidar_back_pose_relative_timestamp = sprintf('%s/%s', data_root_timestamp, file_lidar_back_pose_relative);
    file_lidar_back_pose_relative_sync = sprintf('%s/%s', data_root, file_lidar_back_pose_relative);
    copyfile(file_lidar_back_pose_relative_timestamp, file_lidar_back_pose_relative_sync);
end

%% #2
% sync the lidar point cloud cooresponding to the vehicle pose
if enables{2}
    fprintf('########################### #2 sync the lidar point cloud cooresponding to the vehicle pose   ##########################\n');
    if exist(data_root, 'dir') ~= 7
        fprintf('data_root directory not exist, it will be created!\n');
        mkdir(data_root);
    end

    data_vehicle_pose_synced_dir = sprintf('%s/%s', data_root, dir_vehicle_pose);
    data_lidar_front_synced_dir = sprintf('%s/%s', data_root, dir_lidar_front);
    data_lidar_left_synced_dir = sprintf('%s/%s', data_root, dir_lidar_left);
    data_lidar_right_synced_dir = sprintf('%s/%s', data_root, dir_lidar_right);
    data_lidar_back_synced_dir = sprintf('%s/%s', data_root, dir_lidar_back);

    data_lidar_front_bin_synced_dir = sprintf('%s/%s_bin', data_root, dir_lidar_front);
    data_lidar_left_bin_synced_dir = sprintf('%s/%s_bin', data_root, dir_lidar_left);
    data_lidar_right_bin_synced_dir = sprintf('%s/%s_bin', data_root, dir_lidar_right);
    data_lidar_back_bin_synced_dir = sprintf('%s/%s_bin', data_root, dir_lidar_back);

    if exist(data_vehicle_pose_synced_dir, 'dir') ~= 7
        fprintf('data_vehicle_pose_synced_dir directory not exist, it will be created!\n');
        mkdir(data_vehicle_pose_synced_dir);
    end

    if exist(data_lidar_front_synced_dir, 'dir') ~= 7
        fprintf('data_lidar_front_synced_dir directory not exist, it will be created!\n');
        mkdir(data_lidar_front_synced_dir);
    end

    if exist(data_lidar_left_synced_dir, 'dir') ~= 7
        fprintf('data_lidar_left_synced_dir directory not exist, it will be created!\n');
        mkdir(data_lidar_left_synced_dir);
    end

    if exist(data_lidar_right_synced_dir, 'dir') ~= 7
        fprintf('data_lidar_right_synced_dir directory not exist, it will be created!\n');
        mkdir(data_lidar_right_synced_dir);
    end

    if exist(data_lidar_back_synced_dir, 'dir') ~= 7
        fprintf('data_lidar_back_synced_dir directory not exist, it will be created!\n');
        mkdir(data_lidar_back_synced_dir);
    end

    if exist(data_lidar_front_bin_synced_dir, 'dir') ~= 7
        fprintf('data_lidar_front_bin_synced_dir directory not exist, it will be created!\n');
        mkdir(data_lidar_front_bin_synced_dir);
    end

    if exist(data_lidar_left_bin_synced_dir, 'dir') ~= 7
        fprintf('data_lidar_left_bin_synced_dir directory not exist, it will be created!\n');
        mkdir(data_lidar_left_bin_synced_dir);
    end

    if exist(data_lidar_right_bin_synced_dir, 'dir') ~= 7
        fprintf('data_lidar_right_bin_synced_dir directory not exist, it will be created!\n');
        mkdir(data_lidar_right_bin_synced_dir);
    end

    if exist(data_lidar_back_bin_synced_dir, 'dir') ~= 7
        fprintf('data_lidar_back_bin_synced_dir directory not exist, it will be created!\n');
        mkdir(data_lidar_back_bin_synced_dir);
    end

    % get all the timestamp vehicle file
    data_vehicle_pose_dir = sprintf('%s/%s', data_root_timestamp, dir_vehicle_pose);
    vehicle_pose_filenames_full = dir(fullfile(data_vehicle_pose_dir, '*.txt'));
    vehicle_pose_filenames_full_num = length(vehicle_pose_filenames_full);
    vehicle_pose_filenames_name = cell(vehicle_pose_filenames_full_num, 1);
    for i = 1 : vehicle_pose_filenames_full_num
        vehicle_pose_filenames_name{i} = vehicle_pose_filenames_full(i).name;
    end
    vehicle_pose_filenames_name_sorted = sort(vehicle_pose_filenames_name); % ascend

    % get the lidar front filenames
    data_lidar_front_dir = sprintf('%s/%s', data_root_timestamp, dir_lidar_front);
    lidar_front_filenames_full = dir(fullfile(data_lidar_front_dir, '*.pcd'));
    lidar_front_filenames_full_num = length(lidar_front_filenames_full);
    lidar_front_filenames_name = cell(lidar_front_filenames_full_num, 1);
    lidar_front_filenames_name_number = zeros(lidar_front_filenames_full_num, 1);
    for i = 1 : lidar_front_filenames_full_num
        lidar_front_filenames_name{i} = lidar_front_filenames_full(i).name;
        name_splited = split(lidar_front_filenames_name{i}, '.');
        lidar_front_filenames_name_number(i) = str2num([name_splited{1} '.' name_splited{2}]);
    end

    % get the lidar left filenames
    data_lidar_left_dir = sprintf('%s/%s', data_root_timestamp, dir_lidar_left);
    lidar_left_filenames_full = dir(fullfile(data_lidar_left_dir, '*.pcd'));
    lidar_left_filenames_full_num = length(lidar_left_filenames_full);
    lidar_left_filenames_name = cell(lidar_left_filenames_full_num, 1);
    lidar_left_filenames_name_number = zeros(lidar_left_filenames_full_num, 1);
    for i = 1 : lidar_left_filenames_full_num
        lidar_left_filenames_name{i} = lidar_left_filenames_full(i).name;
        name_splited = split(lidar_left_filenames_name{i}, '.');
        lidar_left_filenames_name_number(i) = str2num([name_splited{1} '.' name_splited{2}]);
    end

    % get the lidar right filenames
    data_lidar_right_dir = sprintf('%s/%s', data_root_timestamp, dir_lidar_right);
    lidar_right_filenames_full = dir(fullfile(data_lidar_right_dir, '*.pcd'));
    lidar_right_filenames_full_num = length(lidar_right_filenames_full);
    lidar_right_filenames_name = cell(lidar_right_filenames_full_num, 1);
    lidar_right_filenames_name_number = zeros(lidar_right_filenames_full_num, 1);
    for i = 1 : lidar_right_filenames_full_num
        lidar_right_filenames_name{i} = lidar_right_filenames_full(i).name;
        name_splited = split(lidar_right_filenames_name{i}, '.');
        lidar_right_filenames_name_number(i) = str2num([name_splited{1} '.' name_splited{2}]);
    end

    % get the lidar back filenames
    data_lidar_back_dir = sprintf('%s/%s', data_root_timestamp, dir_lidar_back);
    lidar_back_filenames_full = dir(fullfile(data_lidar_back_dir, '*.pcd'));
    lidar_back_filenames_full_num = length(lidar_back_filenames_full);
    lidar_back_filenames_name = cell(lidar_back_filenames_full_num, 1);
    lidar_back_filenames_name_number = zeros(lidar_back_filenames_full_num, 1);
    for i = 1 : lidar_back_filenames_full_num
        lidar_back_filenames_name{i} = lidar_back_filenames_full(i).name;
        name_splited = split(lidar_back_filenames_name{i}, '.');
        lidar_back_filenames_name_number(i) = str2num([name_splited{1} '.' name_splited{2}]);
    end

    % search the lidar files to match the timestamp of the vehicle pose
    current_synced_index = start_index;
    for i = 1 : vehicle_pose_filenames_full_num
        current_vehicle_pose_filename = vehicle_pose_filenames_name_sorted{i};
        name_splited = split(current_vehicle_pose_filename, '.');
        current_vehicle_pose_filename_number = str2num([name_splited{1} '.' name_splited{2}]);
        % search the nearest timestamp 
        [lidar_front_sync_offset_min, lidar_front_sync_offset_min_index] = min(abs(current_vehicle_pose_filename_number - lidar_front_filenames_name_number));
        if lidar_front_sync_offset_min < sync_threshold
            lidar_front_synced = true;
        else
            lidar_front_synced = false;
        end

        [lidar_left_sync_offset_min, lidar_left_sync_offset_min_index] = min(abs(current_vehicle_pose_filename_number - lidar_left_filenames_name_number));
        if lidar_left_sync_offset_min < sync_threshold
            lidar_left_synced = true;
        else
            lidar_left_synced = false;
        end

        [lidar_right_sync_offset_min, lidar_right_sync_offset_min_index] = min(abs(current_vehicle_pose_filename_number - lidar_right_filenames_name_number));
        if lidar_right_sync_offset_min < sync_threshold
            lidar_right_synced = true;
        else
            lidar_right_synced = false;
        end

        [lidar_back_sync_offset_min, lidar_back_sync_offset_min_index] = min(abs(current_vehicle_pose_filename_number - lidar_back_filenames_name_number));
        if lidar_back_sync_offset_min < sync_threshold
            lidar_back_synced = true;
        else
            lidar_back_synced = false;
        end

        if lidar_front_synced && lidar_left_synced && lidar_right_synced && lidar_back_synced
            vehicle_pose_filename_timestamp = sprintf('%s/%s', data_vehicle_pose_dir, current_vehicle_pose_filename);
            vehicle_pose_filename_synced = sprintf('%s/%06d.txt', data_vehicle_pose_synced_dir, current_synced_index);
            copyfile(vehicle_pose_filename_timestamp, vehicle_pose_filename_synced);

            lidar_front_filename_timestamp = sprintf('%s/%s', data_lidar_front_dir, lidar_front_filenames_name{lidar_front_sync_offset_min_index});
            lidar_front_filename_synced = sprintf('%s/%06d.pcd', data_lidar_front_synced_dir, current_synced_index);
            copyfile(lidar_front_filename_timestamp, lidar_front_filename_synced);

            lidar_front_bin_filename_synced = sprintf('%s/%06d.bin', data_lidar_front_bin_synced_dir, current_synced_index);
            pcd = pcread(lidar_front_filename_synced);
            fid = fopen(lidar_front_bin_filename_synced, 'wb+');
            fwrite(fid, [pcd.Location pcd.Intensity]', 'single');
            fclose(fid);

            lidar_left_filename_timestamp = sprintf('%s/%s', data_lidar_left_dir, lidar_left_filenames_name{lidar_left_sync_offset_min_index});
            lidar_left_filename_synced = sprintf('%s/%06d.pcd', data_lidar_left_synced_dir, current_synced_index);
            copyfile(lidar_left_filename_timestamp, lidar_left_filename_synced);

            lidar_left_bin_filename_synced = sprintf('%s/%06d.bin', data_lidar_left_bin_synced_dir, current_synced_index);
            pcd = pcread(lidar_left_filename_synced);
            fid = fopen(lidar_left_bin_filename_synced, 'wb+');
            fwrite(fid, [pcd.Location pcd.Intensity]', 'single');
            fclose(fid);

            lidar_right_filename_timestamp = sprintf('%s/%s', data_lidar_right_dir, lidar_right_filenames_name{lidar_right_sync_offset_min_index});
            lidar_right_filename_synced = sprintf('%s/%06d.pcd', data_lidar_right_synced_dir, current_synced_index);
            copyfile(lidar_right_filename_timestamp, lidar_right_filename_synced);

            lidar_right_bin_filename_synced = sprintf('%s/%06d.bin', data_lidar_right_bin_synced_dir, current_synced_index);
            pcd = pcread(lidar_right_filename_synced);
            fid = fopen(lidar_right_bin_filename_synced, 'wb+');
            fwrite(fid, [pcd.Location pcd.Intensity]', 'single');
            fclose(fid);

            lidar_back_filename_timestamp = sprintf('%s/%s', data_lidar_back_dir, lidar_back_filenames_name{lidar_back_sync_offset_min_index});
            lidar_back_filename_synced = sprintf('%s/%06d.pcd', data_lidar_back_synced_dir, current_synced_index);
            copyfile(lidar_back_filename_timestamp, lidar_back_filename_synced);

            lidar_back_bin_filename_synced = sprintf('%s/%06d.bin', data_lidar_back_bin_synced_dir, current_synced_index);
            pcd = pcread(lidar_back_filename_synced);
            fid = fopen(lidar_back_bin_filename_synced, 'wb+');
            fwrite(fid, [pcd.Location pcd.Intensity]', 'single');
            fclose(fid);

            current_synced_index = current_synced_index + 1;
        end

        fprintf('processing the %06d/%06d of the vehicle pose\n', i, vehicle_pose_filenames_full_num);
    end
end