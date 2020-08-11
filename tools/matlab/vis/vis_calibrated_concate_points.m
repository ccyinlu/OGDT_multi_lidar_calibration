% get the trajectory relative from the absolute

if ~exist('data_root')
    data_root = '/media/bingo/SSD/multi_lidar_calib_data/sharingVAN/ch32_4';
end

addpath('../interface');
addpath('../func');
addpath('../thirdParty/yaml');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~exist('calib_before_path')
  calib_before_path = 'calib_before';
end

if ~exist('calib_final_path')
  calib_final_path = 'calib_final';
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

% [roll pitch yaw x y z] [degree degree degree meters meters meters]
lidar_left_to_front_pose_before = loadPoseYaml(sprintf('%s/%s/lidar_1_to_2_estimated_6dof.txt', data_root, calib_before_path));
lidar_right_to_front_pose_before = loadPoseYaml(sprintf('%s/%s/lidar_3_to_2_estimated_6dof.txt', data_root, calib_before_path));
lidar_back_to_front_pose_before = loadPoseYaml(sprintf('%s/%s/lidar_4_to_2_estimated_6dof.txt', data_root, calib_before_path));

transform_matrix_left_to_front_before = transform_matrix_from_pose(lidar_left_to_front_pose_before);
transform_matrix_right_to_front_before = transform_matrix_from_pose(lidar_right_to_front_pose_before);
transform_matrix_back_to_front_before = transform_matrix_from_pose(lidar_back_to_front_pose_before);

% [roll pitch yaw x y z] [degree degree degree meters meters meters]
lidar_left_to_front_pose_final = loadPoseYaml(sprintf('%s/%s/lidar_1_to_2_estimated_6dof.txt', data_root, calib_final_path));
lidar_right_to_front_pose_final = loadPoseYaml(sprintf('%s/%s/lidar_3_to_2_estimated_6dof.txt', data_root, calib_final_path));
lidar_back_to_front_pose_final = loadPoseYaml(sprintf('%s/%s/lidar_4_to_2_estimated_6dof.txt', data_root, calib_final_path));

transform_matrix_left_to_front_final = transform_matrix_from_pose(lidar_left_to_front_pose_final);
transform_matrix_right_to_front_final = transform_matrix_from_pose(lidar_right_to_front_pose_final);
transform_matrix_back_to_front_final = transform_matrix_from_pose(lidar_back_to_front_pose_final);

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
current_lidar_left_points_hom = [current_lidar_left_points ones(size(current_lidar_left_points, 1), 1)];
current_lidar_left_points_transformed_before_hom = (transform_matrix_left_to_front_before * current_lidar_left_points_hom')';
current_lidar_left_points_transformed_before = current_lidar_left_points_transformed_before_hom(:, 1:3);
current_lidar_left_points_transformed_final_hom = (transform_matrix_left_to_front_final * current_lidar_left_points_hom')';
current_lidar_left_points_transformed_final = current_lidar_left_points_transformed_final_hom(:, 1:3);

current_lidar_right_points_hom = [current_lidar_right_points ones(size(current_lidar_right_points, 1), 1)];
current_lidar_right_points_transformed_before_hom = (transform_matrix_right_to_front_before * current_lidar_right_points_hom')';
current_lidar_right_points_transformed_before = current_lidar_right_points_transformed_before_hom(:, 1:3);
current_lidar_right_points_transformed_final_hom = (transform_matrix_right_to_front_final * current_lidar_right_points_hom')';
current_lidar_right_points_transformed_final = current_lidar_right_points_transformed_final_hom(:, 1:3);

current_lidar_back_points_hom = [current_lidar_back_points ones(size(current_lidar_back_points, 1), 1)];
current_lidar_back_points_transformed_before_hom = (transform_matrix_back_to_front_before * current_lidar_back_points_hom')';
current_lidar_back_points_transformed_before = current_lidar_back_points_transformed_before_hom(:, 1:3);
current_lidar_back_points_transformed_final_hom = (transform_matrix_back_to_front_final * current_lidar_back_points_hom')';
current_lidar_back_points_transformed_final = current_lidar_back_points_transformed_final_hom(:, 1:3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pointSize = 2.5;
color_gray_level = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% show the concated points before
limits = [-100 100 -100 100 -100 100];
Ilimits = [0 255];
fig_before = figure();
axes_before = axes(...
                        'Parent', fig_before, ...
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
axis(axes_before, 'equal');
set(fig_before, 'Color', [color_gray_level color_gray_level color_gray_level]);
set(axes_before, 'Color', [color_gray_level color_gray_level color_gray_level]);
set(axes_before, 'Box', 'off');
set(axes_before, 'XColor', [color_gray_level color_gray_level color_gray_level]);
set(axes_before, 'YColor', [color_gray_level color_gray_level color_gray_level]);
set(axes_before, 'ZColor', [color_gray_level color_gray_level color_gray_level]);

pointsShow(axes_before, current_lidar_front_points, limits, Ilimits, 'plain', pointSize, [0.5, 0, 0]);
hold on;
pointsShow(axes_before, current_lidar_left_points_transformed_before, limits, Ilimits, 'plain', pointSize, [0, 0.5, 0]);
hold on;
pointsShow(axes_before, current_lidar_right_points_transformed_before, limits, Ilimits, 'plain', pointSize, [0, 0, 0.5]);
hold on;
pointsShow(axes_before, current_lidar_back_points_transformed_before, limits, Ilimits, 'plain', pointSize, [0.5, 0.5, 0]);
title('concate before calibration');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% show the concated points final
limits = [-100 100 -100 100 -100 100];
Ilimits = [0 255];
fig_final = figure();
axes_final = axes(...
                        'Parent', fig_final, ...
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
axis(axes_final, 'equal');
set(fig_final, 'Color', [color_gray_level color_gray_level color_gray_level]);
set(axes_final, 'Color', [color_gray_level color_gray_level color_gray_level]);
set(axes_final, 'Box', 'off');
set(axes_final, 'XColor', [color_gray_level color_gray_level color_gray_level]);
set(axes_final, 'YColor', [color_gray_level color_gray_level color_gray_level]);
set(axes_final, 'ZColor', [color_gray_level color_gray_level color_gray_level]);

pointsShow(axes_final, current_lidar_front_points, limits, Ilimits, 'plain', pointSize, [0.5, 0, 0]);
hold on;
pointsShow(axes_final, current_lidar_left_points_transformed_final, limits, Ilimits, 'plain', pointSize, [0, 0.5, 0]);
hold on;
pointsShow(axes_final, current_lidar_right_points_transformed_final, limits, Ilimits, 'plain', pointSize, [0, 0, 0.5]);
hold on;
pointsShow(axes_final, current_lidar_back_points_transformed_final, limits, Ilimits, 'plain', pointSize, [0.5, 0.5, 0]);
title('concate after calibration');