function occupancyGridBin = points2OccupancyGridBin(params, input_points)
  % input:
  % params: struct of the points2OccupancyGrid
  % input_points: N x 3 double points
  % params.xRange: half of the side length of the range square along the x axis
  % params.yRange: half of the side length of the range square along the y axis
  % params.resGrid: length of the side square
  % params.zMin: min of z
  % params.zMax: max of z
  % params.levelingMatrix: levelingMatrix to leveling the points, 4 x 4
  % params.Th: threshold for the binarized occupancyGridCounter
  % params.AreaLimit: areaLimit for miniArea remove

  occupancyGrid = points2OccupancyGrid(params, input_points);

  occupancyGridTh = occupancyGrid;
  occupancyGridTh(occupancyGrid > params.Th) = 0;
  occupancyGridTh(occupancyGrid <= params.Th) = 1;

  % open operation to eliminate the noise point
  occupancyGridTh_R = imcomplement(occupancyGridTh);

  % remove the little region
  % search the connected region
  [L, n] = bwlabel(occupancyGridTh_R, 8);
  occupancyGridTh_R_miniLabelRemoved = zeros(size(occupancyGridTh_R));
  for i  = 1 : n
    % calc the pros of the region
    status = regionprops(L == i);
    if status.Area >= params.AreaLimit
      occupancyGridTh_R_miniLabelRemoved(L == i) = 1;
    end
  end

  occupancyGridBin = occupancyGridTh_R_miniLabelRemoved;
end