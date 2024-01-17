classdef constantsCollection
    properties
        n_robots = int8(2); % number of robots
        delta_t = double(1); % time step
        process_noise = eye(1); % process noise for robots
        n_time = 0 % total number of time steps to run
    end
end