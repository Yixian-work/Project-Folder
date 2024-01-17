function[occ_map, source_map, x_r, source] = init_env()
    % occ_map, initialize with obstacle locations
    %occ_map = binaryOccupancyMap(30,30);
    occ_map = robotics.BinaryOccupancyGrid(30,30);
    x = [2;2;2;3;4;7;7;7;8;8;5;5;6;6;(10:13)'];
    y = [3;4;5;5;5;6;7;8;7;5;5;4;2;3;(13:16)'];
    setOccupancy(occ_map, [x y], ones(size(x)));
    show(occ_map)
    % source_map of same size, initialize with equal probability
    %source_map = occupancyMap(30,30);
    source_map = robotics.OccupancyGrid(30,30);
    % Define our initial robot position here
    x_r = [15;15];
    % Define our source location here;
    source = [10 20 15 5;
              20 10 18 10];
end
