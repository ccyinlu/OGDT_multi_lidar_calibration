% get the calibrated extrisic params to leveling the points and then concate the points
if ~exist('data_root')
  data_root = '/media/bingo/SSD/multi_lidar_calib_data/sharingVAN/20200706/lidar_calibration';
  % data_root = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_sync';
end

if ~exist('tools_matlab')
  tools_matlab = '/home/bingo/ethan/multi-lidar-calibration/tools/matlab';
end

addpath([tools_matlab '/' 'interface']);
addpath([tools_matlab '/' 'func']);
addpath([tools_matlab '/' 'thirdParty/yaml']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~exist('calib_after_path')
  % calib_after_path = 'calib_before_leveling';
  calib_after_path = 'planeGridMatch_leveling_calib_final';
end

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

if ~exist('shot_id')
  shot_id = 1;
end

if ~exist('figure_title')
  figure_title = 'calib_after';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [roll pitch yaw x y z] [rad rad rad meters meters meters]
lidar_front_to_front_ground_pose = loadPoseYaml(sprintf('%s/%s/lidar_2_estimated_6dof.txt', data_root, calib_after_path));
lidar_left_to_front_ground_pose = loadPoseYaml(sprintf('%s/%s/lidar_1_estimated_6dof.txt', data_root, calib_after_path));
lidar_right_to_front_ground_pose = loadPoseYaml(sprintf('%s/%s/lidar_3_estimated_6dof.txt', data_root, calib_after_path));
lidar_back_to_front_ground_pose = loadPoseYaml(sprintf('%s/%s/lidar_4_estimated_6dof.txt', data_root, calib_after_path));

transformMatrix_front = eye(4);
transformEulerMatrix_front = eul2rotm([lidar_front_to_front_ground_pose(3) lidar_front_to_front_ground_pose(2) lidar_front_to_front_ground_pose(1)]);
transformMatrix_front(1:3, 1:3) = transformEulerMatrix_front;
transformMatrix_front(1, 4) = lidar_front_to_front_ground_pose(4);
transformMatrix_front(2, 4) = lidar_front_to_front_ground_pose(5);
transformMatrix_front(3, 4) = lidar_front_to_front_ground_pose(6);

transformMatrix_left = eye(4);
transformEulerMatrix_left = eul2rotm([lidar_left_to_front_ground_pose(3) lidar_left_to_front_ground_pose(2) lidar_left_to_front_ground_pose(1)]);
transformMatrix_left(1:3, 1:3) = transformEulerMatrix_left;
transformMatrix_left(1, 4) = lidar_left_to_front_ground_pose(4);
transformMatrix_left(2, 4) = lidar_left_to_front_ground_pose(5);
transformMatrix_left(3, 4) = lidar_left_to_front_ground_pose(6);

transformMatrix_right = eye(4);
transformEulerMatrix_right= eul2rotm([lidar_right_to_front_ground_pose(3) lidar_right_to_front_ground_pose(2) lidar_right_to_front_ground_pose(1)]);
transformMatrix_right(1:3, 1:3) = transformEulerMatrix_right;
transformMatrix_right(1, 4) = lidar_right_to_front_ground_pose(4);
transformMatrix_right(2, 4) = lidar_right_to_front_ground_pose(5);
transformMatrix_right(3, 4) = lidar_right_to_front_ground_pose(6);

transformMatrix_back = eye(4);
transformEulerMatrix_back = eul2rotm([lidar_back_to_front_ground_pose(3) lidar_back_to_front_ground_pose(2) lidar_back_to_front_ground_pose(1)]);
transformMatrix_back(1:3, 1:3) = transformEulerMatrix_back;
transformMatrix_back(1, 4) = lidar_back_to_front_ground_pose(4);
transformMatrix_back(2, 4) = lidar_back_to_front_ground_pose(5);
transformMatrix_back(3, 4) = lidar_back_to_front_ground_pose(6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load the current lidar points
current_lidar_front_pcd_filename = sprintf('%s/%s/%06d.pcd', data_root, lidar_front_path, shot_id);
current_lidar_left_pcd_filename = sprintf('%s/%s/%06d.pcd', data_root, lidar_left_path, shot_id);
current_lidar_right_pcd_filename = sprintf('%s/%s/%06d.pcd', data_root, lidar_right_path, shot_id);
current_lidar_back_pcd_filename = sprintf('%s/%s/%06d.pcd', data_root, lidar_back_path, shot_id);

current_lidar_front_pcd = pcread(current_lidar_front_pcd_filename);
current_lidar_left_pcd = pcread(current_lidar_left_pcd_filename);
current_lidar_right_pcd = pcread(current_lidar_right_pcd_filename);
current_lidar_back_pcd = pcread(current_lidar_back_pcd_filename);

current_lidar_front_points = current_lidar_front_pcd.Location;
current_lidar_left_points = current_lidar_left_pcd.Location;
current_lidar_right_points = current_lidar_right_pcd.Location;
current_lidar_back_points = current_lidar_back_pcd.Location;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% transform the points to concate them
current_lidar_front_points_hom = [current_lidar_front_points ones(size(current_lidar_front_points, 1), 1)];
current_lidar_front_points_transformed_hom = (transformMatrix_front * current_lidar_front_points_hom')';
current_lidar_front_points_transformed = current_lidar_front_points_transformed_hom(:, 1:3);

current_lidar_left_points_hom = [current_lidar_left_points ones(size(current_lidar_left_points, 1), 1)];
current_lidar_left_points_transformed_hom = (transformMatrix_left * current_lidar_left_points_hom')';
current_lidar_left_points_transformed = current_lidar_left_points_transformed_hom(:, 1:3);

current_lidar_right_points_hom = [current_lidar_right_points ones(size(current_lidar_right_points, 1), 1)];
current_lidar_right_points_transformed_hom = (transformMatrix_right * current_lidar_right_points_hom')';
current_lidar_right_points_transformed = current_lidar_right_points_transformed_hom(:, 1:3);

current_lidar_back_points_hom = [current_lidar_back_points ones(size(current_lidar_back_points, 1), 1)];
current_lidar_back_points_transformed_hom = (transformMatrix_back * current_lidar_back_points_hom')';
current_lidar_back_points_transformed = current_lidar_back_points_transformed_hom(:, 1:3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pointSize = 2.5;
color_gray_level = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% show the concated points before
limits = [-100 100 -100 100 -100 100];
Ilimits = [0 255];
fig_concate_leveling = figure();
axes_concate_leveling = axes(...
                  'Parent', fig_concate_leveling, ...
                  'Units', 'Normalized', ...
                  'Position', [0 0 1 1], ...
                  'XTickLabel', '', ...
                  'YTickLabel', '', ...
                  'ZTickLabel', '', ...
                  'XTick', '', ...
                  'YTick', '', ...
                  'ZTick', '', ...
                  'Color', [color_gray_level color_gray_level color_gray_level], ...
                  'PickableParts', 'all', ...
                  'HitTest', 'on', ...
                  'NextPlot', 'replacechildren', ...
                  'Visible', 'on', ...
                  'XLimMode', 'auto', ...
                  'YLimMode', 'auto', ...
                  'ZLimMode', 'auto', ...
                  'DataAspectRatioMode', 'auto', ...
                  'PlotBoxAspectRatioMode', 'auto');

% axis the point view equal
axis(axes_concate_leveling, 'equal');
set(fig_concate_leveling, 'Color', [color_gray_level color_gray_level color_gray_level]);
set(axes_concate_leveling, 'Color', [color_gray_level color_gray_level color_gray_level]);
set(axes_concate_leveling, 'Box', 'off');
set(axes_concate_leveling, 'XColor', [color_gray_level color_gray_level color_gray_level]);
set(axes_concate_leveling, 'YColor', [color_gray_level color_gray_level color_gray_level]);
set(axes_concate_leveling, 'ZColor', [color_gray_level color_gray_level color_gray_level]);

pointsShow(axes_concate_leveling, current_lidar_front_points_transformed, limits, Ilimits, 'plain', pointSize, [0.5, 0, 0]);
hold on;
pointsShow(axes_concate_leveling, current_lidar_left_points_transformed, limits, Ilimits, 'plain', pointSize, [0, 0.5, 0]);
hold on;
pointsShow(axes_concate_leveling, current_lidar_right_points_transformed, limits, Ilimits, 'plain', pointSize, [0, 0, 0.5]);
hold on;
pointsShow(axes_concate_leveling, current_lidar_back_points_transformed, limits, Ilimits, 'plain', pointSize, [0.5, 0.5, 0]);

title(figure_title);