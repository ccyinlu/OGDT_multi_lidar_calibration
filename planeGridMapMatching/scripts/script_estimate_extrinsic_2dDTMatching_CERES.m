% script_estimate_extrinsic_2dDTMatching_CERES
clc;
close all;

if ~exist('data_root')
    % data_root = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_sync';
    data_root = '/media/bingo/SSD/multi_lidar_calib_data/sharingVAN/20200706/lidar_calibration';
end

if ~exist('tools_matlab')
    tools_matlab = '/home/bingo/ethan/multi-lidar-calibration/tools/matlab';
end

addpath('../func');
addpath('../mex');

addpath([tools_matlab '/' 'utils/linefit_ground_segmentation/mex']);
addpath([tools_matlab '/' 'utils/linefit_ground_segmentation/func']);
addpath([tools_matlab '/' 'utils/pointCloud2OccupancyGridMap/mex']);
addpath([tools_matlab '/' 'utils/pointCloud2OccupancyGridMap/func']);
addpath([tools_matlab '/' 'interface']);
addpath([tools_matlab '/' 'thirdParty/yaml']);
addpath([tools_matlab '/' 'func']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~exist('lidar_front_path')
  lidar_front_path = 'lidar_front';
end

if ~exist('lidar_left_path')
  lidar_left_path = 'lidar_left';
end

if ~exist('lidar_right_path')
  lidar_right_path = 'lidar_right';
end

if ~exist('lidar_back_path')
  lidar_back_path = 'lidar_back';
end

if ~exist('init_6dof_path')
  init_6dof_path = 'calib_before_leveling';
end

if ~exist('est_6dof_path')
  est_6dof_path = 'planeGridMatch_leveling_calib_final';
end

if ~exist('shot_id')
  shot_id = 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% params for ground plane detection
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% params for occupancy grid map generation
if ~exist('xRange')
  xRange = 80;
end

if ~exist('yRange')
  yRange = 80;
end

if ~exist('resGrid')
  resGrid = 0.2;
end

if ~exist('zMin')
  zMin = 0.5;
end

if ~exist('zMax')
  zMax = 5;
end

if ~exist('Th')
  Th = 1; % 6
end

if ~exist('AreaLimit')
  AreaLimit = 1; % 8
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% load the single shot points cloud
current_lidar_front_points_filename = sprintf('%s/%s/%06d.pcd', data_root, lidar_front_path, shot_id);
current_lidar_left_points_filename = sprintf('%s/%s/%06d.pcd', data_root, lidar_left_path, shot_id);
current_lidar_right_points_filename = sprintf('%s/%s/%06d.pcd', data_root, lidar_right_path, shot_id);
current_lidar_back_points_filename = sprintf('%s/%s/%06d.pcd', data_root, lidar_back_path, shot_id);

current_lidar_front_points_pc = pcread(current_lidar_front_points_filename);
current_lidar_left_points_pc = pcread(current_lidar_left_points_filename);
current_lidar_right_points_pc = pcread(current_lidar_right_points_filename);
current_lidar_back_points_pc = pcread(current_lidar_back_points_filename);

% the data must be double
current_lidar_front_points = double(current_lidar_front_points_pc.Location);
current_lidar_left_points = double(current_lidar_left_points_pc.Location);
current_lidar_right_points = double(current_lidar_right_points_pc.Location);
current_lidar_back_points = double(current_lidar_back_points_pc.Location);

% remove inf from the raw points
current_lidar_front_points = removeInf(current_lidar_front_points);
current_lidar_left_points = removeInf(current_lidar_left_points);
current_lidar_right_points = removeInf(current_lidar_right_points);
current_lidar_back_points = removeInf(current_lidar_back_points);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% I: estimate the ground plane to estimate the delta_z, delta_roll, delta_pitch
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% estimate delta_zrp from mount_zrp (fail)
% % estimated the mount_z, mount_roll, mount_pitch params using the segmented plane
% [mount_z_front, mount_pitch_front, mount_roll_front] = linefit_ground_estimation(groundSegmentParams, current_lidar_front_points);
% [mount_z_left, mount_pitch_left, mount_roll_left] = linefit_ground_estimation(groundSegmentParams, current_lidar_left_points);
% [mount_z_right, mount_pitch_right, mount_roll_right] = linefit_ground_estimation(groundSegmentParams, current_lidar_right_points);
% [mount_z_back, mount_pitch_back, mount_roll_back] = linefit_ground_estimation(groundSegmentParams, current_lidar_back_points);

% % estimate the delta_z, delta_roll, delta_pitch from the left, right and back to front according to the mount_z, mount_roll, mount_pitch
% deltaZRP_left2front = estimate_deltaZRP_from_mountZRP([mount_z_left mount_roll_left mount_pitch_left], ...
%                                                       [mount_z_front mount_roll_front mount_pitch_front]);
% deltaZRP_right2front = estimate_deltaZRP_from_mountZRP([mount_z_right mount_roll_right mount_pitch_right], ...
%                                                       [mount_z_front mount_roll_front mount_pitch_front]);
% deltaZRP_back2front = estimate_deltaZRP_from_mountZRP([mount_z_back mount_roll_back mount_pitch_back], ...
%                                                       [mount_z_front mount_roll_front mount_pitch_front]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% estimate delta_zrp from ground plane (fail)
%% estimate the ground plane
% lidar_front_plane_equation = ground_plane_estimation(groundSegmentParams, current_lidar_front_points);
% lidar_left_plane_equation = ground_plane_estimation(groundSegmentParams, current_lidar_left_points);
% lidar_right_plane_equation = ground_plane_estimation(groundSegmentParams, current_lidar_right_points);
% lidar_back_plane_equation = ground_plane_estimation(groundSegmentParams, current_lidar_back_points);

%% estimate the delta_z, delta_roll, delta_pitch from the left, right and back to front according to the plane equation
% deltaZRP_left2front = estimate_deltaZRP_from_groundPlane(lidar_left_plane_equation, lidar_front_plane_equation);
% deltaZRP_right2front = estimate_deltaZRP_from_groundPlane(lidar_right_plane_equation, lidar_front_plane_equation);
% deltaZRP_back2front = estimate_deltaZRP_from_groundPlane(lidar_back_plane_equation, lidar_front_plane_equation);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% estimate delta_x_y_yaw from occupancy grid map

% #1: estimate the plane and leveling the pointcloud
% segment the leveling input_point_cloud using the ransac estimated ground plane
[mount_z_front, mount_pitch_front, mount_roll_front] = linefit_ground_estimation(groundSegmentParams, current_lidar_front_points);
[mount_z_left, mount_pitch_left, mount_roll_left] = linefit_ground_estimation(groundSegmentParams, current_lidar_left_points);
[mount_z_right, mount_pitch_right, mount_roll_right] = linefit_ground_estimation(groundSegmentParams, current_lidar_right_points);
[mount_z_back, mount_pitch_back, mount_roll_back] = linefit_ground_estimation(groundSegmentParams, current_lidar_back_points);

% get the leveling_matrix
levelingMatrix_front = eye(4);
levelingEulerMatrix_front = eul2rotm([0 mount_pitch_front/180*pi mount_roll_front/180*pi]);
levelingMatrix_front(1:3, 1:3) = levelingEulerMatrix_front;
levelingMatrix_front(3, 4) = mount_z_front;

levelingMatrix_left = eye(4);
levelingEulerMatrix_left = eul2rotm([0 mount_pitch_left/180*pi mount_roll_left/180*pi]);
levelingMatrix_left(1:3, 1:3) = levelingEulerMatrix_left;
levelingMatrix_left(3, 4) = mount_z_left;

levelingMatrix_right = eye(4);
levelingEulerMatrix_right = eul2rotm([0 mount_pitch_right/180*pi mount_roll_right/180*pi]);
levelingMatrix_right(1:3, 1:3) = levelingEulerMatrix_right;
levelingMatrix_right(3, 4) = mount_z_right;

levelingMatrix_back = eye(4);
levelingEulerMatrix_back = eul2rotm([0 mount_pitch_back/180*pi mount_roll_back/180*pi]);
levelingMatrix_back(1:3, 1:3) = levelingEulerMatrix_back;
levelingMatrix_back(3, 4) = mount_z_back;

% #2: convert the pointcloud to occupancy grid map
points2OccupancyGridParams = struct();
points2OccupancyGridParams.xRange = xRange;
points2OccupancyGridParams.yRange = yRange;
points2OccupancyGridParams.resGrid = resGrid;
points2OccupancyGridParams.Th = Th;
points2OccupancyGridParams.AreaLimit = AreaLimit;

points2OccupancyGridParams_front = points2OccupancyGridParams;
% points2OccupancyGridParams_front.zMin = zMin - mount_z_front;
% points2OccupancyGridParams_front.zMax = zMax - mount_z_front;
points2OccupancyGridParams_front.zMin = zMin;
points2OccupancyGridParams_front.zMax = zMax;
points2OccupancyGridParams_front.levelingMatrix = levelingMatrix_front;

points2OccupancyGridParams_left = points2OccupancyGridParams;
% points2OccupancyGridParams_left.zMin = zMin - mount_z_left;
% points2OccupancyGridParams_left.zMax = zMax - mount_z_left;
points2OccupancyGridParams_left.zMin = zMin;
points2OccupancyGridParams_left.zMax = zMax;
points2OccupancyGridParams_left.levelingMatrix = levelingMatrix_left;

points2OccupancyGridParams_right = points2OccupancyGridParams;
% points2OccupancyGridParams_right.zMin = zMin - mount_z_right;
% points2OccupancyGridParams_right.zMax = zMax - mount_z_right;
points2OccupancyGridParams_right.zMin = zMin;
points2OccupancyGridParams_right.zMax = zMax;
points2OccupancyGridParams_right.levelingMatrix = levelingMatrix_right;

points2OccupancyGridParams_back = points2OccupancyGridParams;
% points2OccupancyGridParams_back.zMin = zMin - mount_z_back;
% points2OccupancyGridParams_back.zMax = zMax - mount_z_back;
points2OccupancyGridParams_back.zMin = zMin;
points2OccupancyGridParams_back.zMax = zMax;
points2OccupancyGridParams_back.levelingMatrix = levelingMatrix_back;

% occupancyGridBin, higher value means obstacles; occupancyGridBin_R: lower value means obstacles
occupancyGridBin_front = points2OccupancyGridBin(points2OccupancyGridParams_front, current_lidar_front_points);
occupancyGridBin_R_front = imcomplement(occupancyGridBin_front);
occupancyGridBin_left = points2OccupancyGridBin(points2OccupancyGridParams_left, current_lidar_left_points);
occupancyGridBin_R_left = imcomplement(occupancyGridBin_left);
occupancyGridBin_right = points2OccupancyGridBin(points2OccupancyGridParams_right, current_lidar_right_points);
occupancyGridBin_R_right = imcomplement(occupancyGridBin_right);
occupancyGridBin_back = points2OccupancyGridBin(points2OccupancyGridParams_back, current_lidar_back_points);
occupancyGridBin_R_back = imcomplement(occupancyGridBin_back);

% figure();
% imshow(occupancyGridBin_R_front);
% figure();
% imshow(occupancyGridBin_R_left);
% figure();
% imshow(occupancyGridBin_R_right);
% figure();
% imshow(occupancyGridBin_R_back);

% #3: convert the left, right, back occupancy grid map to laserscan
map_struct = struct();
map_struct.resGrid = resGrid;
map_laserscan_front = convert_occupancyGridMap_to_laserscan(map_struct, occupancyGridBin_front);
map_laserscan_left = convert_occupancyGridMap_to_laserscan(map_struct, occupancyGridBin_left);
map_laserscan_right = convert_occupancyGridMap_to_laserscan(map_struct, occupancyGridBin_right);
map_laserscan_back = convert_occupancyGridMap_to_laserscan(map_struct, occupancyGridBin_back);

% figure();
% plot(map_laserscan_front);
% figure();
% plot(map_laserscan_left);
% figure();
% plot(map_laserscan_right);
% figure();
% plot(map_laserscan_back);

% #4: matching the laserscan to occupancy grid map to get the [x y yaw]

% load the init 6dof params
%% load the init_6dof
init_6dof_left_to_front_filename = sprintf('%s/%s/lidar_1_estimated_6dof.txt', data_root, init_6dof_path);
init_6dof_right_to_front_filename = sprintf('%s/%s/lidar_3_estimated_6dof.txt', data_root, init_6dof_path);
init_6dof_back_to_front_filename = sprintf('%s/%s/lidar_4_estimated_6dof.txt', data_root, init_6dof_path);

% [roll pitch yaw x y z] [rad rad rad m m m]
init_6dof_left_to_front = loadPoseYaml(init_6dof_left_to_front_filename);
init_6dof_right_to_front = loadPoseYaml(init_6dof_right_to_front_filename);
init_6dof_back_to_front = loadPoseYaml(init_6dof_back_to_front_filename);

% generate the map_dt
[map_dt_front, ~] = bwdist(occupancyGridBin_front);
[map_dt_left, ~] = bwdist(occupancyGridBin_left);
[map_dt_right, ~] = bwdist(occupancyGridBin_right);
[map_dt_back, ~] = bwdist(occupancyGridBin_back);

map_dt_front_reduced = map_dt_front.^(1/2);
map_dt_left_reduced = map_dt_left.^(1/2);
map_dt_right_reduced = map_dt_right.^(1/2);
map_dt_back_reduced = map_dt_back.^(1/2);

% figure();
% imshow(map_dt_front_reduced/max(map_dt_front_reduced(:)));
% figure();
% imshow(map_dt_left_reduced/max(map_dt_left_reduced(:)));
% figure();
% imshow(map_dt_right_reduced/max(map_dt_right_reduced(:)));
% figure();
% imshow(map_dt_back_reduced/max(map_dt_back_reduced(:)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get the overlapped left with the front based on the init 6dof
laserscan_overlapping_params_left_to_front = struct();
laserscan_overlapping_params_left_to_front.initPose = [
  init_6dof_left_to_front(4) ...
  init_6dof_left_to_front(5) ...
  init_6dof_left_to_front(3)
];
laserscan_overlapping_params_left_to_front.sectorRes = overlapping_sectorRes;
laserscan_overlapping_params_left_to_front.overlappingThChild = overlappingThChild;
laserscan_overlapping_params_left_to_front.overlappingThParent = overlappingThParent;

[overlapped_laserscan_xy_left_to_front, ~] = laserscanSectorOverlappingTransformed(laserscan_overlapping_params_left_to_front, ...
                                                                                                        map_laserscan_left.Cartesian, ...
                                                                                                        map_laserscan_front.Cartesian);


% estimate the extrinsic params according to the grid map mathcing
estimateExtrinsicParams_left2front = struct();
cur2ref_pose_guess_left2front = [init_6dof_left_to_front(4) init_6dof_left_to_front(5) init_6dof_left_to_front(3)];
estimateExtrinsicParams_left2front.initExtrinsicParams = cur2ref_pose_guess_left2front; % [x y yaw] [m m rad]
estimateExtrinsicParams_left2front.verbose = true;
estimateExtrinsicParams_left2front.cauchy_c = 5;

estimate_map_struct = struct();
estimate_map_struct.resolution = resGrid;
map_origin = zeros(1, 2);
map_origin(1) = -resGrid * size(map_dt_front, 2) / 2;
map_origin(2) = -resGrid * size(map_dt_front, 1) / 2;
estimate_map_struct.origin = map_origin;
% [x y yaw] [m m rad]
[Params_left2front, dt_estimationErrors_left2front] = estimate_extrinsic_2dDTMatching_CERES( map_dt_front, ...
                                                                    overlapped_laserscan_xy_left_to_front, ...
                                                                    estimate_map_struct, ...
                                                                    estimateExtrinsicParams_left2front);

dt_loss_guess_left2front = calc_dt_loss(map_dt_front, overlapped_laserscan_xy_left_to_front, cur2ref_pose_guess_left2front, estimate_map_struct);
dt_loss_estimate_left2front = calc_dt_loss(map_dt_front, overlapped_laserscan_xy_left_to_front, Params_left2front.opt, estimate_map_struct);
fprintf('cur2ref_pose_guess_left2front: [%.4f, %.4f, %.4f], dt_loss_guess_left2front: %.4f\n', ...
    cur2ref_pose_guess_left2front(1), ...
    cur2ref_pose_guess_left2front(2), ...
    cur2ref_pose_guess_left2front(3), ...
    dt_loss_guess_left2front);

fprintf('ceres estimate: [%.4f, %.4f, %.4f], dt_loss_est_left2front: %.4f\n', ...
    Params_left2front.opt(1), ...
    Params_left2front.opt(2), ...
    Params_left2front.opt(3), ...
    dt_loss_estimate_left2front);

cur_left2front_laserscan_init_vu_final = points2d_project2map(map_laserscan_left.Cartesian, cur2ref_pose_guess_left2front, estimate_map_struct, size(map_dt_front, 2), size(map_dt_front, 1));
cur_left2front_laserscan_estimated_vu_final = points2d_project2map(map_laserscan_left.Cartesian, Params_left2front.opt, estimate_map_struct, size(map_dt_front, 2), size(map_dt_front, 1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get the overlapped right with the front based on the init 6dof
laserscan_overlapping_params_right_to_front = struct();
laserscan_overlapping_params_right_to_front.initPose = [
  init_6dof_right_to_front(4) ...
  init_6dof_right_to_front(5) ...
  init_6dof_right_to_front(3)
];
laserscan_overlapping_params_right_to_front.sectorRes = overlapping_sectorRes;
laserscan_overlapping_params_right_to_front.overlappingThChild = overlappingThChild;
laserscan_overlapping_params_right_to_front.overlappingThParent = overlappingThParent;

[overlapped_laserscan_xy_right_to_front, ~] = laserscanSectorOverlappingTransformed(laserscan_overlapping_params_right_to_front, ...
                                                                                                        map_laserscan_right.Cartesian, ...
                                                                                                        map_laserscan_front.Cartesian);


% estimate the extrinsic params according to the grid map mathcing
estimateExtrinsicParams_right2front = struct();
cur2ref_pose_guess_right2front = [init_6dof_right_to_front(4) init_6dof_right_to_front(5) init_6dof_right_to_front(3)];
estimateExtrinsicParams_right2front.initExtrinsicParams = cur2ref_pose_guess_right2front; % [x y yaw] [m m rad]
estimateExtrinsicParams_right2front.verbose = true;
estimateExtrinsicParams_right2front.cauchy_c = 5;

estimate_map_struct = struct();
estimate_map_struct.resolution = resGrid;
map_origin = zeros(1, 2);
map_origin(1) = -resGrid * size(map_dt_front, 2) / 2;
map_origin(2) = -resGrid * size(map_dt_front, 1) / 2;
estimate_map_struct.origin = map_origin;
[Params_right2front, dt_estimationErrors_right2front] = estimate_extrinsic_2dDTMatching_CERES( map_dt_front, ...
                                                                    overlapped_laserscan_xy_right_to_front, ...
                                                                    estimate_map_struct, ...
                                                                    estimateExtrinsicParams_right2front);

dt_loss_guess_right2front = calc_dt_loss(map_dt_front, overlapped_laserscan_xy_right_to_front, cur2ref_pose_guess_right2front, estimate_map_struct);
dt_loss_estimate_right2front = calc_dt_loss(map_dt_front, overlapped_laserscan_xy_right_to_front, Params_right2front.opt, estimate_map_struct);
fprintf('cur2ref_pose_guess_right2front: [%.4f, %.4f, %.4f], dt_loss_guess_right2front: %.4f\n', ...
    cur2ref_pose_guess_right2front(1), ...
    cur2ref_pose_guess_right2front(2), ...
    cur2ref_pose_guess_right2front(3), ...
    dt_loss_guess_right2front);

fprintf('ceres estimate: [%.4f, %.4f, %.4f], dt_loss_est_right2front: %.4f\n', ...
    Params_right2front.opt(1), ...
    Params_right2front.opt(2), ...
    Params_right2front.opt(3), ...
    dt_loss_estimate_right2front);

cur_right2front_laserscan_init_vu_final = points2d_project2map(map_laserscan_right.Cartesian, cur2ref_pose_guess_right2front, estimate_map_struct, size(map_dt_front, 2), size(map_dt_front, 1));
cur_right2front_laserscan_estimated_vu_final = points2d_project2map(map_laserscan_right.Cartesian, Params_right2front.opt, estimate_map_struct, size(map_dt_front, 2), size(map_dt_front, 1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get the overlapped back with the front based on the init 6dof

%% first extend the front laserscan and reform the 2d occupancy grid map for the front
map_laserscan_left_xy_transformed = points2d_transform(map_laserscan_left.Cartesian, Params_left2front.opt);
map_laserscan_right_xy_transformed = points2d_transform(map_laserscan_right.Cartesian, Params_right2front.opt);
map_laserscan_front_xy_extented = [map_laserscan_front.Cartesian; map_laserscan_left_xy_transformed; map_laserscan_right_xy_transformed];
map_laserscan_front_extented = lidarScan(map_laserscan_front_xy_extented);

occupancyGridBin_front_extended = laserscan2OccupancyGridMex(points2OccupancyGridParams_front, map_laserscan_front_xy_extented);

% flipup the original occupancyGridMap
occupancyGridBin_front_extended = flipud(occupancyGridBin_front_extended);

occupancyGridBin_R_front_extended = imcomplement(occupancyGridBin_front_extended);
[map_dt_front_extended, ~] = bwdist(occupancyGridBin_front_extended);

laserscan_overlapping_params_back_to_front = struct();
laserscan_overlapping_params_back_to_front.initPose = [
  init_6dof_back_to_front(4) ...
  init_6dof_back_to_front(5) ...
  init_6dof_back_to_front(3)
];
laserscan_overlapping_params_back_to_front.sectorRes = overlapping_sectorRes;
laserscan_overlapping_params_back_to_front.overlappingThChild = overlappingThChild;
laserscan_overlapping_params_back_to_front.overlappingThParent = overlappingThParent;

[overlapped_laserscan_xy_back_to_front, ~] = laserscanSectorOverlappingTransformed(laserscan_overlapping_params_back_to_front, ...
                                                                                                        map_laserscan_back.Cartesian, ...
                                                                                                        map_laserscan_front_extented.Cartesian);


% estimate the extrinsic params according to the grid map mathcing
estimateExtrinsicParams_back2front = struct();
cur2ref_pose_guess_back2front = [init_6dof_back_to_front(4) init_6dof_back_to_front(5) init_6dof_back_to_front(3)];
estimateExtrinsicParams_back2front.initExtrinsicParams = cur2ref_pose_guess_back2front; % [x y yaw] [m m rad]
estimateExtrinsicParams_back2front.verbose = true;
estimateExtrinsicParams_back2front.cauchy_c = 5;

estimate_map_struct = struct();
estimate_map_struct.resolution = resGrid;
map_origin = zeros(1, 2);
map_origin(1) = -resGrid * size(map_dt_front_extended, 2) / 2;
map_origin(2) = -resGrid * size(map_dt_front_extended, 1) / 2;
estimate_map_struct.origin = map_origin;
[Params_back2front, dt_estimationErrors_back2front] = estimate_extrinsic_2dDTMatching_CERES( map_dt_front_extended, ...
                                                                    overlapped_laserscan_xy_back_to_front, ...
                                                                    estimate_map_struct, ...
                                                                    estimateExtrinsicParams_back2front);

dt_loss_guess_back2front = calc_dt_loss(map_dt_front_extended, overlapped_laserscan_xy_back_to_front, cur2ref_pose_guess_back2front, estimate_map_struct);
dt_loss_estimate_back2front = calc_dt_loss(map_dt_front_extended, overlapped_laserscan_xy_back_to_front, Params_back2front.opt, estimate_map_struct);
fprintf('cur2ref_pose_guess_back2front: [%.4f, %.4f, %.4f], dt_loss_guess_back2front: %.4f\n', ...
    cur2ref_pose_guess_back2front(1), ...
    cur2ref_pose_guess_back2front(2), ...
    cur2ref_pose_guess_back2front(3), ...
    dt_loss_guess_back2front);

fprintf('ceres estimate: [%.4f, %.4f, %.4f], dt_loss_est_back2front: %.4f\n', ...
    Params_back2front.opt(1), ...
    Params_back2front.opt(2), ...
    Params_back2front.opt(3), ...
    dt_loss_estimate_back2front);

cur_back2front_laserscan_init_vu_final = points2d_project2map(map_laserscan_back.Cartesian, cur2ref_pose_guess_back2front, estimate_map_struct, size(map_dt_front_extended, 2), size(map_dt_front_extended, 1));
cur_back2front_laserscan_estimated_vu_final = points2d_project2map(map_laserscan_back.Cartesian, Params_back2front.opt, estimate_map_struct, size(map_dt_front_extended, 2), size(map_dt_front_extended, 1));

figure();
subplot(2,1,1);
imshow(occupancyGridBin_R_front);
hold on;
scatter(cur_left2front_laserscan_init_vu_final(:, 1), cur_left2front_laserscan_init_vu_final(:, 2), 8, [1 0 0], 'filled'); 
hold on;
scatter(cur_right2front_laserscan_init_vu_final(:, 1), cur_right2front_laserscan_init_vu_final(:, 2), 8, [1 0 0], 'filled'); 
hold on;
scatter(cur_back2front_laserscan_init_vu_final(:, 1), cur_back2front_laserscan_init_vu_final(:, 2), 8, [1 0 0], 'filled'); 
title('init');

subplot(2,1,2);
imshow(occupancyGridBin_R_front_extended);
hold on;
scatter(cur_left2front_laserscan_estimated_vu_final(:, 1), cur_left2front_laserscan_estimated_vu_final(:, 2), 8, [1 0 0], 'filled'); 
hold on;
scatter(cur_right2front_laserscan_estimated_vu_final(:, 1), cur_right2front_laserscan_estimated_vu_final(:, 2), 8, [1 0 0], 'filled'); 
hold on;
scatter(cur_back2front_laserscan_estimated_vu_final(:, 1), cur_back2front_laserscan_estimated_vu_final(:, 2), 8, [1 0 0], 'filled'); 
title('estimated');

% output the leveling results
estimated_6dof_results_dir = sprintf('%s/%s', data_root, est_6dof_path);
if ~exist(estimated_6dof_results_dir)
  command = sprintf('mkdir -p %s', estimated_6dof_results_dir);
  system(command);
end

% [roll pitch yaw x y z]
% [roll pitch z] refers the roll, pitch and z offset to the ground
% [x y yaw] refers to the left, right and back refers to the front lidar
estimated_6dof_front_to_ground_front_filename = sprintf('%s/lidar_2_estimated_6dof.txt', estimated_6dof_results_dir);
outputPoseYaml([mount_roll_front/180*pi; ...
                mount_pitch_front/180*pi; ...
                0; ...
                0; ...
                0; ...
                mount_z_front], ...
                estimated_6dof_front_to_ground_front_filename);

estimated_6dof_left_to_ground_front_filename = sprintf('%s/lidar_1_estimated_6dof.txt', estimated_6dof_results_dir);
outputPoseYaml([mount_roll_left/180*pi; ...
                mount_pitch_left/180*pi; ...
                Params_left2front.opt(3); ...
                Params_left2front.opt(1); ...
                Params_left2front.opt(2); ...
                mount_z_left], ...
                estimated_6dof_left_to_ground_front_filename);

estimated_6dof_right_to_ground_front_filename = sprintf('%s/lidar_3_estimated_6dof.txt', estimated_6dof_results_dir);
outputPoseYaml([mount_roll_right/180*pi; ...
                mount_pitch_right/180*pi; ...
                Params_right2front.opt(3); ...
                Params_right2front.opt(1); ...
                Params_right2front.opt(2); ...
                mount_z_right], ...
                estimated_6dof_right_to_ground_front_filename);

estimated_6dof_back_to_ground_front_filename = sprintf('%s/lidar_4_estimated_6dof.txt', estimated_6dof_results_dir);
outputPoseYaml([mount_roll_back/180*pi; ...
                mount_pitch_back/180*pi; ...
                Params_back2front.opt(3); ...
                Params_back2front.opt(1); ...
                Params_back2front.opt(2); ...
                mount_z_back], ...
                estimated_6dof_back_to_ground_front_filename);