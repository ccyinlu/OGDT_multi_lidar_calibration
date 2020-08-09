function cur_laserScan_xy_transformed = points2d_transform(cur_laserScan_xy, transform_pose)
    cur_laserScan_xy_hom = [cur_laserScan_xy ones(size(cur_laserScan_xy, 1), 1)];
    tranformation_matrix = [
        cos(transform_pose(3)) -sin(transform_pose(3)) transform_pose(1);
        sin(transform_pose(3)) cos(transform_pose(3)) transform_pose(2);
                      0                   0                   1;
    ];
    cur_laserscan_transformed_xy_hom = (tranformation_matrix * cur_laserScan_xy_hom')';
    cur_laserScan_xy_transformed = cur_laserscan_transformed_xy_hom(:, 1:2);
end