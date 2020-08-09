function cur_laserscan_transformed_vu_final = points2d_project2map(cur_laserScan_xy, cur2map_pose, map_struct, map_width, map_height)
    cur_laserScan_xy_hom = [cur_laserScan_xy ones(size(cur_laserScan_xy, 1), 1)];
    tranformation_matrix = [
        cos(cur2map_pose(3)) -sin(cur2map_pose(3)) cur2map_pose(1);
        sin(cur2map_pose(3)) cos(cur2map_pose(3)) cur2map_pose(2);
        0                   0                   1;
    ];
    cur_laserscan_transformed_xy_hom = (tranformation_matrix * cur_laserScan_xy_hom')';
    cur_laserscan_transformed_xy = cur_laserscan_transformed_xy_hom(:, 1:2);

    map_origin = map_struct.origin;
    map_res = map_struct.resolution;
    cur_laserscan_transformed_vu = (cur_laserscan_transformed_xy - map_origin(1:2)) / map_res;

    cur_laserscan_transformed_vu(:, 2) = map_height - cur_laserscan_transformed_vu(:, 2);

    index = cur_laserscan_transformed_vu(:,1) > 1 & ...
            cur_laserscan_transformed_vu(:,1) < map_width & ...
            cur_laserscan_transformed_vu(:,2) > 1 & ...
            cur_laserscan_transformed_vu(:,2) < map_height; 

    cur_laserscan_transformed_vu_final = cur_laserscan_transformed_vu(index, :);
end