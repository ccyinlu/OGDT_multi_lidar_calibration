% get the trajectory file from pose relative

clc;
close all;

addpath('../../interface');
addpath('../../func');
addpath('../../thirdParty/yaml');

if ~exist('data_root')
    data_root = '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt_sync';
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

if ~exist('vehicle_trajectory_absolute')
    vehicle_trajectory_absolute = 'vehicle_trajectory_absolute.txt';
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

if ~exist('vehicle_trajectory_relative')
    vehicle_trajectory_relative = 'vehicle_trajectory_relative.txt';
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

if ~exist('vehicle_pose_dir')
    vehicle_pose_dir = 'vehicle_pose';
end

if ~exist('start_index')
    start_index = 1;
end

vehicle_pose_data_dir = sprintf('%s/%s', data_root, vehicle_pose_dir);
observation_num = length(dir(fullfile(vehicle_pose_data_dir, '*.txt')));

vehicle_trajectory_absolute_filename = sprintf('%s/%s', data_root, vehicle_trajectory_absolute);
lidar_front_trajectory_absolute_filename = sprintf('%s/%s', data_root, lidar_front_trajectory_absolute);
lidar_left_trajectory_absolute_filename = sprintf('%s/%s', data_root, lidar_left_trajectory_absolute);
lidar_right_trajectory_absolute_filename = sprintf('%s/%s', data_root, lidar_right_trajectory_absolute);
lidar_back_trajectory_absolute_filename = sprintf('%s/%s', data_root, lidar_back_trajectory_absolute);

vehicle_trajectory_relative_filename = sprintf('%s/%s', data_root, vehicle_trajectory_relative);
lidar_front_trajectory_relative_filename = sprintf('%s/%s', data_root, lidar_front_trajectory_relative);
lidar_left_trajectory_relative_filename = sprintf('%s/%s', data_root, lidar_left_trajectory_relative);
lidar_right_trajectory_relative_filename = sprintf('%s/%s', data_root, lidar_right_trajectory_relative);
lidar_back_trajectory_relative_filename = sprintf('%s/%s', data_root, lidar_back_trajectory_relative);

% load the pose_relative
lidar_front_pose_to_vehicle = loadPoseYaml([data_root '/' file_lidar_front_pose_relative]);
lidar_left_pose_to_vehicle = loadPoseYaml([data_root '/' file_lidar_left_pose_relative]);
lidar_right_pose_to_vehicle = loadPoseYaml([data_root '/' file_lidar_right_pose_relative]);
lidar_back_pose_to_vehicle = loadPoseYaml([data_root '/' file_lidar_back_pose_relative]);

transform_matrix_front_to_vehicle = transform_matrix_from_pose(lidar_front_pose_to_vehicle);
transform_matrix_left_to_vehicle = transform_matrix_from_pose(lidar_left_pose_to_vehicle);
transform_matrix_right_to_vehicle = transform_matrix_from_pose(lidar_right_pose_to_vehicle);
transform_matrix_back_to_vehicle = transform_matrix_from_pose(lidar_back_pose_to_vehicle);

% load vehicle pose and generate the lidar absolute pose
fid_vehicle_trajectory_absolute = fopen(vehicle_trajectory_absolute_filename, 'w');
fid_lidar_front_trajectory_absolute = fopen(lidar_front_trajectory_absolute_filename, 'w');
fid_lidar_left_trajectory_absolute = fopen(lidar_left_trajectory_absolute_filename, 'w');
fid_lidar_right_trajectory_absolute = fopen(lidar_right_trajectory_absolute_filename, 'w');
fid_lidar_back_trajectory_absolute = fopen(lidar_back_trajectory_absolute_filename, 'w');

% frame-to-frame relative pose
fid_vehicle_trajectory_relative = fopen(vehicle_trajectory_relative_filename, 'w');
fid_lidar_front_trajectory_relative = fopen(lidar_front_trajectory_relative_filename, 'w');
fid_lidar_left_trajectory_relative = fopen(lidar_left_trajectory_relative_filename, 'w');
fid_lidar_right_trajectory_relative = fopen(lidar_right_trajectory_relative_filename, 'w');
fid_lidar_back_trajectory_relative = fopen(lidar_back_trajectory_relative_filename, 'w');

for i = start_index : observation_num
    current_vehicle_pose_absolute = loadPoseYaml(sprintf('%s/%06d.txt', vehicle_pose_data_dir, i));
    current_vehicle_tranform_matrix_absolute = transform_matrix_from_pose(current_vehicle_pose_absolute);

    % front_pose = vehicle_pose * front_to_vehicle
    current_lidar_front_transform_matrix_absolute = current_vehicle_tranform_matrix_absolute * transform_matrix_front_to_vehicle;
    current_lidar_front_pose_absolute = pose_from_transform_matrix(current_lidar_front_transform_matrix_absolute);

    % left_pose = vehicle_pose * left_to_vehicle
    current_lidar_left_transform_matrix_absolute = current_vehicle_tranform_matrix_absolute * transform_matrix_left_to_vehicle;
    current_lidar_left_pose_absolute = pose_from_transform_matrix(current_lidar_left_transform_matrix_absolute);

    % right_pose = vehicle_pose * right_to_vehicle
    current_lidar_right_transform_matrix_absolute = current_vehicle_tranform_matrix_absolute * transform_matrix_right_to_vehicle;
    current_lidar_right_pose_absolute = pose_from_transform_matrix(current_lidar_right_transform_matrix_absolute);

    % back_pose = vehicle_pose * back_to_vehicle
    current_lidar_back_transform_matrix_absolute = current_vehicle_tranform_matrix_absolute * transform_matrix_back_to_vehicle;
    current_lidar_back_pose_absolute = pose_from_transform_matrix(current_lidar_back_transform_matrix_absolute);

    % save the current absolute pose, [x y z roll pitch yaw] in radius
    str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                    current_vehicle_pose_absolute(4), ...
                    current_vehicle_pose_absolute(5), ...
                    current_vehicle_pose_absolute(6), ...
                    current_vehicle_pose_absolute(1), ...
                    current_vehicle_pose_absolute(2), ...
                    current_vehicle_pose_absolute(3));
    fprintf(fid_vehicle_trajectory_absolute, str);
    str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                current_lidar_front_pose_absolute(4), ...
                current_lidar_front_pose_absolute(5), ...
                current_lidar_front_pose_absolute(6), ...
                current_lidar_front_pose_absolute(1), ...
                current_lidar_front_pose_absolute(2), ...
                current_lidar_front_pose_absolute(3));
    fprintf(fid_lidar_front_trajectory_absolute, str);
    str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                current_lidar_left_pose_absolute(4), ...
                current_lidar_left_pose_absolute(5), ...
                current_lidar_left_pose_absolute(6), ...
                current_lidar_left_pose_absolute(1), ...
                current_lidar_left_pose_absolute(2), ...
                current_lidar_left_pose_absolute(3));
    fprintf(fid_lidar_left_trajectory_absolute, str);
    str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                current_lidar_right_pose_absolute(4), ...
                current_lidar_right_pose_absolute(5), ...
                current_lidar_right_pose_absolute(6), ...
                current_lidar_right_pose_absolute(1), ...
                current_lidar_right_pose_absolute(2), ...
                current_lidar_right_pose_absolute(3));
    fprintf(fid_lidar_right_trajectory_absolute, str);
    str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                current_lidar_back_pose_absolute(4), ...
                current_lidar_back_pose_absolute(5), ...
                current_lidar_back_pose_absolute(6), ...
                current_lidar_back_pose_absolute(1), ...
                current_lidar_back_pose_absolute(2), ...
                current_lidar_back_pose_absolute(3));
    fprintf(fid_lidar_back_trajectory_absolute, str);

    % generate the absolute pose
    if i == start_index
        previous_vehicle_tranform_matrix_absolute = current_vehicle_tranform_matrix_absolute;
        previous_lidar_front_transform_matrix_absolute = current_lidar_front_transform_matrix_absolute;
        previous_lidar_left_transform_matrix_absolute = current_lidar_left_transform_matrix_absolute;
        previous_lidar_right_transform_matrix_absolute = current_lidar_right_transform_matrix_absolute;
        previous_lidar_back_transform_matrix_absolute = current_lidar_back_transform_matrix_absolute;
    end

    % current = previous * current_to_previous
    current_vehicle_tranform_matrix_relative = inv(previous_vehicle_tranform_matrix_absolute) * current_vehicle_tranform_matrix_absolute;
    current_vehicle_pose_relative = pose_from_transform_matrix(current_vehicle_tranform_matrix_relative);

    current_lidar_front_transform_matrix_relative = inv(previous_lidar_front_transform_matrix_absolute) * current_lidar_front_transform_matrix_absolute;
    current_lidar_front_pose_relative = pose_from_transform_matrix(current_lidar_front_transform_matrix_relative);

    current_lidar_left_transform_matrix_relative = inv(previous_lidar_left_transform_matrix_absolute) * current_lidar_left_transform_matrix_absolute;
    current_lidar_left_pose_relative = pose_from_transform_matrix(current_lidar_left_transform_matrix_relative);

    current_lidar_right_transform_matrix_relative = inv(previous_lidar_right_transform_matrix_absolute) * current_lidar_right_transform_matrix_absolute;
    current_lidar_right_pose_relative = pose_from_transform_matrix(current_lidar_right_transform_matrix_relative);

    current_lidar_back_transform_matrix_relative = inv(previous_lidar_back_transform_matrix_absolute) * current_lidar_back_transform_matrix_absolute;
    current_lidar_back_pose_relative = pose_from_transform_matrix(current_lidar_back_transform_matrix_relative);

    previous_vehicle_tranform_matrix_absolute = current_vehicle_tranform_matrix_absolute;
    previous_lidar_front_transform_matrix_absolute = current_lidar_front_transform_matrix_absolute;
    previous_lidar_left_transform_matrix_absolute = current_lidar_left_transform_matrix_absolute;
    previous_lidar_right_transform_matrix_absolute = current_lidar_right_transform_matrix_absolute;
    previous_lidar_back_transform_matrix_absolute = current_lidar_back_transform_matrix_absolute;

    % save the current relative pose, [x y z roll pitch yaw] in radius
    str = sprintf('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
                current_vehicle_pose_relative(4), ...
                current_vehicle_pose_relative(5), ...
                current_vehicle_pose_relative(6), ...
                current_vehicle_pose_relative(1), ...
                current_vehicle_pose_relative(2), ...
                current_vehicle_pose_relative(3));
    fprintf(fid_vehicle_trajectory_relative, str);
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

    fprintf('processing frame %06d/%06d\n', i, observation_num);
end

fclose(fid_vehicle_trajectory_absolute);
fclose(fid_lidar_front_trajectory_absolute);
fclose(fid_lidar_left_trajectory_absolute);
fclose(fid_lidar_right_trajectory_absolute);
fclose(fid_lidar_back_trajectory_absolute);

fclose(fid_vehicle_trajectory_relative);
fclose(fid_lidar_front_trajectory_relative);
fclose(fid_lidar_left_trajectory_relative);
fclose(fid_lidar_right_trajectory_relative);
fclose(fid_lidar_back_trajectory_relative);
