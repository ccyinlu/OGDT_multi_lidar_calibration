% test linefit segmentation
close all;
clc;

addpath('../mex');
addpath('../func');

if ~exist('data_raw_root')
  % data_raw_root = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_sync';
  data_raw_root = '/media/bingo/SSD/multi_lidar_calib_data/sharingVAN/20200620/lidar_calibration';
end

if ~exist('tools_root')
  tools_root = '/home/bingo/ethan/multi-lidar-calibration/tools/matlab';
end

addpath([tools_root '/' 'utils/linefit_ground_segmentation/func']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~exist('lidar_type')
  lidar_type = 'right';
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
  resGrid = 0.3;
end

if ~exist('zMin')
  zMin = 0.5;
end

if ~exist('zMax')
  zMax = 5;
end

if ~exist('Th')
  Th = 2;
end

point_cloud_dir = sprintf('%s/lidar_%s', data_raw_root, lidar_type);
point_cloud_filename = sprintf('%s/%06d.pcd', point_cloud_dir, data_index);
input_point_cloud_pt = pcread(point_cloud_filename);
input_point_cloud = double(input_point_cloud_pt.Location);

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% segment the leveling input_point_cloud using the ransac estimated ground plane
[mount_z, mount_pitch, mount_roll] = linefit_ground_estimation(groundSegmentParams, input_point_cloud);
groundSegmentParams.leveling = true;
groundSegmentParams.levelingPreset = true;
groundSegmentParams.levelingPresetZ = mount_z;
groundSegmentParams.levelingPresetRoll = mount_roll/180*pi; % degree
groundSegmentParams.levelingPresetPitch = mount_pitch/180*pi; % degree

% get the leveling_matrix
levelingMatrix = eye(4);
levelingEulerMatrix = eul2rotm([0 groundSegmentParams.levelingPresetPitch groundSegmentParams.levelingPresetRoll]);
levelingMatrix(1:3, 1:3) = levelingEulerMatrix;

points2OccupancyGridParams = struct();
points2OccupancyGridParams.xRange = xRange;
points2OccupancyGridParams.yRange = yRange;
points2OccupancyGridParams.resGrid = resGrid;
points2OccupancyGridParams.zMin = zMin - mount_z;
points2OccupancyGridParams.zMax = zMax - mount_z;
points2OccupancyGridParams.levelingMatrix = levelingMatrix;

occupancyGrid = points2OccupancyGrid(points2OccupancyGridParams, input_point_cloud);

% normalized the occupancyGrid
occupancyGridNormalized = occupancyGrid/max(occupancyGrid(:));
occupancyGridNormalizedReverted = 1 - occupancyGridNormalized;

occupancyGridTh = occupancyGrid;
occupancyGridTh(occupancyGrid > Th) = 0;
occupancyGridTh(occupancyGrid <= Th) = 1;

% open operation to eliminate the noise point
occupancyGridTh_R = imcomplement(occupancyGridTh);

% remove the little region
% search the connected region
[L, n] = bwlabel(occupancyGridTh_R, 8);
occupancyGridTh_R_miniLabelRemoved = zeros(size(occupancyGridTh_R));
areaLimit = 10;
for i  = 1 : n
  % calc the pros of the region
  status = regionprops(L == i);
  if status.Area >= areaLimit
    occupancyGridTh_R_miniLabelRemoved(L == i) = 1;
  end
end

occupancyGridTh_miniLabelRemoved = imcomplement(occupancyGridTh_R_miniLabelRemoved);

figure();
pcshow(pointCloud(input_point_cloud));

figure();
imshow(occupancyGridNormalizedReverted);

figure();
imshow(occupancyGridTh);

figure();
imshow(occupancyGridTh_miniLabelRemoved);
