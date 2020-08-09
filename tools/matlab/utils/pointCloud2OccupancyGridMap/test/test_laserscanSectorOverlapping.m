% test linefit segmentation
close all;
clc;

addpath('../mex');
addpath('../func');

if ~exist('data_raw_root')
  data_raw_root = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_sync';
  % data_raw_root = '/media/bingo/SSD/multi_lidar_calib_data/sharingVAN/20200620/lidar_calibration';
end

if ~exist('tools_root')
  tools_root = '/home/bingo/ethan/multi-lidar-calibration/tools/matlab';
end

addpath([tools_root '/' 'utils/linefit_ground_segmentation/func']);
addpath([tools_root '/' 'utils/linefit_ground_segmentation/mex']);
addpath([tools_root '/' 'interface']);
addpath([tools_root '/' 'thirdParty/yaml']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~exist('lidar_parent')
  lidar_parent = 'lidar_front';
end

if ~exist('lidar_child')
  lidar_child = 'lidar_left';
end

if ~exist('init_6dof_path')
  init_6dof_path = 'calib_before';
end

if ~exist('data_index')
  data_index = 1;
end

if ~exist('r_min')
  r_min = 0.2;
end

if ~exist('r_max')
  r_max = 100;
end

if ~exist('n_bins')
  n_bins = 360;
end

if ~exist('n_segments')
  n_segments = 360;
end

if ~exist('max_dist_to_line')
  max_dist_to_line = 0.05;
end

if ~exist('max_slope')
  max_slope = 0.3;
end

if ~exist('max_fit_error')
  max_fit_error = 0.02; % 0.05
end

if ~exist('long_threshold')
  long_threshold = 1.0;
end

if ~exist('max_long_height')
  max_long_height = 0.1;
end

if ~exist('max_start_height')
  max_start_height = 0.2;
end

if ~exist('sensor_height')
  sensor_height = 0.8;
end

if ~exist('line_search_angle')
  line_search_angle = 0.1;
end

if ~exist('n_threads')
  n_threads = 4;
end

if ~exist('leveling')
  leveling = false;
end

if ~exist('levelingPreset')
  levelingPreset = false;
end

if ~exist('levelingPresetZ')
  levelingPresetZ = 0;
end

if ~exist('levelingPresetPitch')
  levelingPresetPitch = 0;
end

if ~exist('levelingPresetRoll')
  levelingPresetRoll = 0;
end

if ~exist('xRange')
  xRange = 80;
end

if ~exist('yRange')
  yRange = 80;
end

if ~exist('resGrid')
  resGrid = 0.5;
end

if ~exist('zMin')
  zMin = 0.5;
end

if ~exist('zMax')
  zMax = 5;
end

if ~exist('Th')
  Th = 6;
end

if ~exist('AreaLimit')
  AreaLimit = 8;
end

if ~exist('overlapping_sectorRes')
  overlapping_sectorRes = 1; % with unit degree
end

if ~exist('overlappingThChild')
  overlappingThChild = 1; % counter
end

if ~exist('overlappingThParent')
  overlappingThParent = 1; % counter
end

point_cloud_dir_parent = sprintf('%s/%s', data_raw_root, lidar_parent);
point_cloud_filename_parent = sprintf('%s/%06d.pcd', point_cloud_dir_parent, data_index);
input_point_cloud_pt_parent = pcread(point_cloud_filename_parent);
input_point_cloud_parent = double(input_point_cloud_pt_parent.Location);

point_cloud_dir_child = sprintf('%s/%s', data_raw_root, lidar_child);
point_cloud_filename_child = sprintf('%s/%06d.pcd', point_cloud_dir_child, data_index);
input_point_cloud_pt_child = pcread(point_cloud_filename_child);
input_point_cloud_child = double(input_point_cloud_pt_child.Location);

groundSegmentParams = struct();
groundSegmentParams.r_min_square = double(r_min * r_min);
groundSegmentParams.r_max_square = double(r_max * r_max);
groundSegmentParams.n_bins = double(n_bins);
groundSegmentParams.n_segments = double(n_segments);
groundSegmentParams.max_dist_to_line = double(max_dist_to_line);
groundSegmentParams.max_slope = double(max_slope);
groundSegmentParams.max_error_square = double(max_fit_error * max_fit_error);
groundSegmentParams.long_threshold = double(long_threshold);
groundSegmentParams.max_long_height = double(max_long_height);
groundSegmentParams.max_start_height = double(max_start_height);
groundSegmentParams.sensor_height = double(sensor_height);
groundSegmentParams.line_search_angle = double(line_search_angle);
groundSegmentParams.n_threads = double(n_threads);
groundSegmentParams.leveling = leveling;
groundSegmentParams.levelingPreset = levelingPreset;
groundSegmentParams.levelingPresetZ = levelingPresetZ;
groundSegmentParams.levelingPresetPitch = levelingPresetPitch;
groundSegmentParams.levelingPresetRoll = levelingPresetRoll;

points2OccupancyGridParams = struct();
points2OccupancyGridParams.xRange = xRange;
points2OccupancyGridParams.yRange = yRange;
points2OccupancyGridParams.resGrid = resGrid;
points2OccupancyGridParams.Th = Th;
points2OccupancyGridParams.AreaLimit = AreaLimit;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% segment the leveling input_point_cloud_parent using the ransac estimated ground plane
[mount_z_parent, mount_pitch_parent, mount_roll_parent] = linefit_ground_estimation(groundSegmentParams, input_point_cloud_parent);
groundSegmentParams.leveling = true;
groundSegmentParams.levelingPreset = true;
groundSegmentParams.levelingPresetZ = mount_z_parent;
groundSegmentParams.levelingPresetPitch = mount_pitch_parent/180*pi; % degree
groundSegmentParams.levelingPresetRoll = mount_roll_parent/180*pi; % degree

points2OccupancyGridParams.zMin = zMin - mount_z_parent;
points2OccupancyGridParams.zMax = zMax - mount_z_parent;

% get the leveling_matrix
levelingMatrix = eye(4);
levelingEulerMatrix = eul2rotm([0 groundSegmentParams.levelingPresetPitch groundSegmentParams.levelingPresetRoll]);
levelingMatrix(1:3, 1:3) = levelingEulerMatrix;

points2OccupancyGridParams.levelingMatrix = levelingMatrix;

occupancyGridBin_parent = points2OccupancyGridBin(points2OccupancyGridParams, input_point_cloud_parent);

occupancyGridBin_R_parent = imcomplement(occupancyGridBin_parent);

figure();
pcshow(pointCloud(input_point_cloud_parent));

figure();
imshow(occupancyGridBin_R_parent);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% segment the leveling input_point_cloud_child using the ransac estimated ground plane
[mount_z_child, mount_pitch_child, mount_roll_child] = linefit_ground_estimation(groundSegmentParams, input_point_cloud_child);
groundSegmentParams.leveling = true;
groundSegmentParams.levelingPreset = true;
groundSegmentParams.levelingPresetZ = mount_z_child;
groundSegmentParams.levelingPresetPitch = mount_pitch_child/180*pi; % degree
groundSegmentParams.levelingPresetRoll = mount_roll_child/180*pi; % degree

points2OccupancyGridParams.zMin = zMin - mount_z_child;
points2OccupancyGridParams.zMax = zMax - mount_z_child;

% get the leveling_matrix
levelingMatrix = eye(4);
levelingEulerMatrix = eul2rotm([0 groundSegmentParams.levelingPresetPitch groundSegmentParams.levelingPresetRoll]);
levelingMatrix(1:3, 1:3) = levelingEulerMatrix;

points2OccupancyGridParams.levelingMatrix = levelingMatrix;

occupancyGridBin_child = points2OccupancyGridBin(points2OccupancyGridParams, input_point_cloud_child);

occupancyGridBin_R_child = imcomplement(occupancyGridBin_child);

figure();
pcshow(pointCloud(input_point_cloud_child));

figure();
imshow(occupancyGridBin_R_child);

% load the init 6dof params
%% load the init_6dof
if isequal(lidar_child, 'lidar_left')
  init_6dof_child_to_parent_filename = sprintf('%s/%s/lidar_1_to_2_estimated_6dof.txt', data_raw_root, init_6dof_path);
elseif isequal(lidar_child, 'lidar_right')
  init_6dof_child_to_parent_filename = sprintf('%s/%s/lidar_3_to_2_estimated_6dof.txt', data_raw_root, init_6dof_path);
elseif isequal(lidar_child, 'lidar_back')
  init_6dof_child_to_parent_filename = sprintf('%s/%s/lidar_4_to_2_estimated_6dof.txt', data_raw_root, init_6dof_path);
end

% [roll pitch yaw x y z] [rad rad rad m m m]
init_6dof_child_to_parent = loadPoseYaml(init_6dof_child_to_parent_filename);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% convert the occupancyGridMap to laserscan
map_struct = struct();
map_struct.resGrid = resGrid;
laserscan_child = convert_occupancyGridMap_to_laserscan(map_struct, occupancyGridBin_child);
laserscan_parent = convert_occupancyGridMap_to_laserscan(map_struct, occupancyGridBin_parent);

laserscan_overlapping_params = struct();
laserscan_overlapping_params.initPose = [
  init_6dof_child_to_parent(4) ...
  init_6dof_child_to_parent(5) ...
  init_6dof_child_to_parent(3)
];
laserscan_overlapping_params.sectorRes = overlapping_sectorRes;
laserscan_overlapping_params.overlappingThChild = overlappingThChild;
laserscan_overlapping_params.overlappingThParent = overlappingThParent;

%% get the overlapped points according to initPose

[overlapped_laserscan_xy_child, overlapped_laserscan_xy_parent] = laserscanSectorOverlappingTransformed(laserscan_overlapping_params, ...
                                                                                                laserscan_child.Cartesian, ...
                                                                                                laserscan_parent.Cartesian);

initPose = laserscan_overlapping_params.initPose;
transform_matrix_child_2_parent = [
  cos(initPose(3)) -sin(initPose(3)) initPose(1);
  sin(initPose(3))  cos(initPose(3)) initPose(2);
          0                  0            1
];

overlapped_laserscan_xy_child_hom = [overlapped_laserscan_xy_child ones(size(overlapped_laserscan_xy_child, 1), 1)];
overlapped_laserscan_xy_child_transformed_hom = (transform_matrix_child_2_parent * overlapped_laserscan_xy_child_hom')';
overlapped_laserscan_xy_child_transformed = overlapped_laserscan_xy_child_transformed_hom(:, 1:2);

laserscan_xy_child_hom = [laserscan_child.Cartesian ones(size(laserscan_child.Cartesian, 1), 1)];
laserscan_xy_child_transformed_hom = (transform_matrix_child_2_parent * laserscan_xy_child_hom')';
laserscan_xy_child_transformed = laserscan_xy_child_transformed_hom(:, 1:2);

figure();
plot(laserscan_parent);
hold on;
plot(lidarScan(laserscan_xy_child_transformed));
hold on;
plot(lidarScan(overlapped_laserscan_xy_child_transformed));
hold on;
plot(lidarScan(overlapped_laserscan_xy_parent));