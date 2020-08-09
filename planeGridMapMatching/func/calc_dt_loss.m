function dt_loss = calc_dt_loss(map_dt, cur_laserScan_xy, cur2map_pose, map_struct)

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
    cur_laserscan_transformed_uv = (cur_laserscan_transformed_xy - map_origin(1:2)) / map_res;

    map_width = size(map_dt, 2);
    map_height = size(map_dt, 2);

    cur_laserscan_transformed_uv(:, 2) = map_height - cur_laserscan_transformed_uv(:, 2);

    index = cur_laserscan_transformed_uv(:,1) > 1 & ...
            cur_laserscan_transformed_uv(:,1) < map_width & ...
            cur_laserscan_transformed_uv(:,2) > 1 & ...
            cur_laserscan_transformed_uv(:,2) < map_height; 

    cur_laserscan_transformed_uv_final = cur_laserscan_transformed_uv(index, :);

    dt_loss = sum(map_dt(sub2ind([map_height, map_width], floor(cur_laserscan_transformed_uv_final(:,2)), floor(cur_laserscan_transformed_uv_final(:,1)))));
    dt_loss = dt_loss + 10000 * sum(~index);
    dt_loss = dt_loss / size(cur_laserscan_transformed_uv, 1);
end