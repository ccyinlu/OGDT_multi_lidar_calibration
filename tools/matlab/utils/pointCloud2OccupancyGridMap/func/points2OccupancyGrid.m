function occupancyGrid = points2OccupancyGrid(params, input_points)
  % input:
  % params: struct of the points2OccupancyGrid
  % input_points: N x 3 double points
  % params.xRange: half of the side length of the range square along the x axis
  % params.yRange: half of the side length of the range square along the y axis
  % params.resGrid: length of the side square
  % params.zMin: min of z
  % params.zMax: max of z
  % params.levelingMatrix: levelingMatrix to leveling the points, 4 x 4

  % first leveling the points
  levelingMatrix = params.levelingMatrix;
  input_point_cloud_hom = [input_points ones(size(input_points, 1), 1)]; % N x 4
  leveling_input_point_cloud_hom = (levelingMatrix * input_point_cloud_hom')'; % N x 4
  leveling_input_points = leveling_input_point_cloud_hom(:, 1:3);

  occupancyGrid = levelingPoints2OccupancyGridMex(params, leveling_input_points);
  
  % flip the occupancyGrid
  occupancyGrid = flipud(occupancyGrid);
end