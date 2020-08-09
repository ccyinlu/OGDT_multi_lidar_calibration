function [overlapped_points3d_child, overlapped_points3d_parent] = points3dSectorOverlappingTransformed(params, ...
                                                                                      child_points3d, ...
                                                                                      parent_points3d)

  % input:
  % params: struct of the overlapping
  % params.initMatrix: init matrix from the child to parent, 4x4
  % params.sectorRes: angle resolution of the sector
  % params.overlappingThChild: threshold of the overlapping of the child
  % params.overlappingThParent: threshold of the overlapping of the parent
  % child_points3d: Nx3
  % parent_points3d: Nx3

  % output:
  % overlapped_points3d_child: overlappedChildPointsNum x 3
  % overlapped_points3d_parent: overlappedParentPointsNum x 3

  % form the tranformation matrix from the child to the parent according to the init pose
  transform_matrix_child_2_parent = params.initMatrix;

  transform_matrix_parent_2_child = inv(transform_matrix_child_2_parent);

  % transform the points from the child frame to the parent frame
  child_points3d_hom = [child_points3d ones(size(child_points3d, 1), 1)];
  child_points3d_transformed_hom = (transform_matrix_child_2_parent * child_points3d_hom')';
  child_points3d_transformed = child_points3d_transformed_hom(:, 1:3);

  [overlapped_points3d_child_transformed, overlapped_points3d_parent] = points3dSectorOverlappingMex(params, child_points3d_transformed, parent_points3d);

  overlapped_points3d_child_transformed_hom = [overlapped_points3d_child_transformed ones(size(overlapped_points3d_child_transformed, 1), 1)];
  overlapped_points3d_child_hom = (transform_matrix_parent_2_child * overlapped_points3d_child_transformed_hom')';
  overlapped_points3d_child = overlapped_points3d_child_hom(:, 1:3);
end