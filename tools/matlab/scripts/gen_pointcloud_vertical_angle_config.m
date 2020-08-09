% generate the vetical angle of the pointCloud

clc;
close all;

addpath('../utils/cloudSegmentation/func');
addpath('../utils/cloudSegmentation/mex');

pointCloudBase_file_name = '/media/bingo/CCYINLU/DATA/sharingVAN/sharing-van-1.0plus/2#/2020-04-30/lidar1_000001.pcd';

pointCloudBase = pcread(pointCloudBase_file_name);

N = 32;

params = struct();
points = [pointCloudBase.Location pointCloudBase.Intensity];

vertical_theta = zeros(1, N);

vertical_theta = vertical_theta/180*pi;

% set the params
params.N_SCAN = N;
params.Horizon_SCAN = 1800;
params.vertical_theta = vertical_theta;

% you should convert the points from single to double, if you use the pcread, then the point type will be single
[~, ~, ~, v_angle_raw] = projectPointCloudMex(params, double(points));

v_angle_raw_deg = v_angle_raw * 180 / pi;
 
% kmeans
v_angle_raw_deg_id = kmeans(v_angle_raw_deg, params.N_SCAN);

v_angle_config = zeros(params.N_SCAN, 1);

for i = 1 : params.N_SCAN
    v_angle_raw_deg_sel = v_angle_raw_deg(v_angle_raw_deg_id == i);
    v_angle_config(i) = mean(v_angle_raw_deg_sel);
end

v_angle_config_descend = sort(v_angle_config, 'descend');
