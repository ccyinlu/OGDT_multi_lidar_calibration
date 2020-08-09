function occupancyGridMapDt = gen_occupancyGridMap_DT(occupancyGridMap)
    % occupancyGridMap: zeros means free, higher value means obstacle

    occupancyGridMap_bin = zeros(size(occupancyGridMap, 1), size(occupancyGridMap, 2));
    occupancyGridMap_bin(occupancyGridMap >= 1) = 1;

    [occupancyGridMapDt, ~] = bwdist(occupancyGridMap_bin);
end