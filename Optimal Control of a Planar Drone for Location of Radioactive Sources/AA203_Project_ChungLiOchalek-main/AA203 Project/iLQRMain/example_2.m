%% Example code for getReadingObstacle.m
%% Single obstacle
% Drone is at (0, 0)
y = 0;
z = 0;

% Source is at (20, 0) with k = 1
y_ans = 20;
z_ans = 0;
k = 1;

% Obstacle is a circle with center at (10, 0) with radius 2 and mu = 1
obstacles = [10, 0, 2, 1];

% Final reading
S = getReadingObstacle(y, z, y_ans, z_ans, k, obstacles);

%% Two obstacles
% Obstacles are two circles, one with center at (10, 0) with radius 2 and
% mu = 1, the other with center at (2, 0) with radius 1 and mu = 2
obstacles = [10, 0, 2, 1; 2, 0, 1, 2];

% Final reading
S = getReadingObstacle(y, z, y_ans, z_ans, k, obstacles);