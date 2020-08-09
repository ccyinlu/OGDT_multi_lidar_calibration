function [overlapped_laserscan_xy_child, overlapped_laserscan_xy_parent] = laserscanSectorOverlappingTransformed(params, ...
                                                                                      child_laserscan_xy, ...
                                                                                      parent_laserscan_xy)

  % input:
  % params: struct of the overlapping
  % params.initPose: init pose from the child to parent, [x y yaw] [m m rad]
  % params.sectorRes: angle resolution of the sector
  % params.overlappingThChild: threshold of the overlapping of the child
  % params.overlappingThParent: threshold of the overlapping of the parent
  % child_laserscan_xy: Nx2
  % parent_laserscan_xy: Nx2

  % output:
  % sectorCounter: sectorNum x 2
  % overlappingFlagChild: childPointsNum x 1
  % overlappingFlagParent: parentPointsNum x 1

  % form the tranformation matrix from the child to the parent according to the init pose
  initPose = params.initPose;
  transform_matrix_child_2_parent = [
    cos(initPose(3)) -sin(initPose(3)) initPose(1);
    sin(initPose(3))  cos(initPose(3)) initPose(2);
           0                  0            1
  ];

  transform_matrix_parent_2_child = inv(transform_matrix_child_2_parent);

  % transform the points from the child frame to the parent frame
  child_laserscan_xy_hom = [child_laserscan_xy ones(size(child_laserscan_xy, 1), 1)];
  child_laserscan_xy_transformed_hom = (transform_matrix_child_2_parent * child_laserscan_xy_hom')';
  child_laserscan_xy_transformed = child_laserscan_xy_transformed_hom(:, 1:2);

  [sectorNum, overlappingFlagChildTransformed, overlappingFlagParent] = laserscanSectorOverlappingMex(params, child_laserscan_xy_transformed, parent_laserscan_xy);

  overlapped_laserscan_xy_child_transformed = child_laserscan_xy_transformed(overlappingFlagChildTransformed == 1, :);
  overlapped_laserscan_xy_parent = parent_laserscan_xy(overlappingFlagParent == 1, :);

  overlapped_laserscan_xy_child_transformed_hom = [overlapped_laserscan_xy_child_transformed ones(size(overlapped_laserscan_xy_child_transformed, 1), 1)];
  overlapped_laserscan_xy_child_hom = (transform_matrix_parent_2_child * overlapped_laserscan_xy_child_transformed_hom')';
  overlapped_laserscan_xy_child = overlapped_laserscan_xy_child_hom(:, 1:2);
end