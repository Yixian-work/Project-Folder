function[occ_map, source_map, x_r, source] = init_env()
    dim = 30;
    % occ_map, initialize with obstacle locations
    %occ_map = binaryOccupancyMap(30,30);
    occ_map = robotics.BinaryOccupancyGrid(dim,dim);
    %x = [2;2;2;3;4;7;7;7;8;8;5;5;6;6;(10:13)'];
    x = randi(dim,80,1);
    y = randi(dim,80,1);
    %y = [3;4;5;5;5;6;7;8;7;5;5;4;2;3;(13:16)'];
    setOccupancy(occ_map, [x y], ones(size(x)));
    show(occ_map);
    % source_map of same size, initialize with equal probability
    %source_map = occupancyMap(30,30);
    %source_map = robotics.OccupancyGrid(30,30);
    source_map = robotics.OccupancyGrid(.2*ones(dim));
    % Define our initial robot position here
    x_r = [10, 15, 25;
           15, 15, 5];
    x_r = [14, 15, 17;
           15, 16, 17];
    % Define our source location here;
    source = randi(dim,2,6);
    %source = [10 20 15 5;
    %          20 10 18 10];
end
