function [overlapped_laserscan_xy_child, overlapped_laserscan_xy_parent] = laserscanSectorOverlapping(params, ...
                                                                                      child_laserscan_xy, ...
                                                                                      parent_laserscan_xy)

  % input:
  % params: struct of the overlapping
  % params.sectorRes: angle resolution of the sector with uinit degree
  % params.overlappingThChild: threshold of the overlapping of the child
  % params.overlappingThParent: threshold of the overlapping of the parent
  % child_laserscan_xy: Nx2
  % parent_laserscan_xy: Nx2

  % output:
  % sectorCounter: sectorNum x 2
  % overlappingFlagChild: childPointsNum x 1
  % overlappingFlagParent: parentPointsNum x 1

  [~, overlappingFlagChild, overlappingFlagParent] = laserscanSectorOverlappingMex(params, child_laserscan_xy, parent_laserscan_xy);

  overlapped_laserscan_xy_child = child_laserscan_xy(overlappingFlagChild, :);
  overlapped_laserscan_xy_parent = parent_laserscan_xy(overlappingFlagParent, :);
end