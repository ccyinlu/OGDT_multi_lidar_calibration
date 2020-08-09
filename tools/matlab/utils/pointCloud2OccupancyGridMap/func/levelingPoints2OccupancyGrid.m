function occupancyGrid = levelingPoints2OccupancyGrid(params, input_leveling_points)
  % input:
  % params: struct of the levelingPoints2OccupancyGrid
  % input_leveling_points: N x 3 double points
  % params.xRange: half of the side length of the range square along the x axis
  % params.yRange: half of the side length of the range square along the y axis
  % params.resGrid: length of the side square
  % params.zMin: min of z
  % params.zMax: max of z
  occupancyGrid = levelingPoints2OccupancyGridMex(params, input_leveling_points);
end