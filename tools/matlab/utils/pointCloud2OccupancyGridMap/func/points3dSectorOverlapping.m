function [overlapped_points3d_child, overlapped_points3d_parent] = points3dSectorOverlapping(params, ...
                                                                                      child_points3d, ...
                                                                                      parent_points3d)

  % input:
  % params: struct of the overlapping
  % params.sectorRes: angle resolution of the sector with uinit degree
  % params.overlappingThChild: threshold of the overlapping of the child
  % params.overlappingThParent: threshold of the overlapping of the parent
  % child_points3d: Nx3
  % parent_points3d: Nx3

  % output:
  % overlapped_points3d_child: overlappedChildPointsNum x 3
  % overlapped_points3d_parent: overlappedParentPointsNum x 3

  [overlapped_points3d_child, overlapped_points3d_parent] = points3dSectorOverlappingMex(params, child_points3d, parent_points3d);
end