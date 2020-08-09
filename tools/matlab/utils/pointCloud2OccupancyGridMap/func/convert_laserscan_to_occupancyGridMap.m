function occupancyGridMap = convert_laserscan_to_occupancyGridMap(params, map_laserscan)
  % params: struct of the params
  % params.resGrid
  % params.xRange
  % params.yRange

  resGrid_ = params.resGrid;
  xRange_ = params.xRange;
  yRange_ = params.yRange;

  occupancyGridMap_width = size(occupancyGridMap, 2);
  occupancyGridMap_height = size(occupancyGridMap, 1);

  map_origin = zeros(2, 1);
  map_origin(1) = -resGrid_ * occupancyGridMap_width / 2;
  map_origin(2) = -resGrid_ * occupancyGridMap_height / 2;

  % get all the non-zero intensity pixel (u,v) location
  index_obstacles = find(occupancyGridMap >=1);
  u_obstacles = mod(index_obstacles, occupancyGridMap_height) + 1;
  v_obstacles = floor(index_obstacles/occupancyGridMap_height) + 1;
  laserscan_x = v_obstacles * resGrid_ + map_origin(1);
  laserscan_y = (occupancyGridMap_height - u_obstacles + 1) * resGrid_ + map_origin(2);

  map_laserscan = lidarScan([laserscan_x laserscan_y]);
end