% this script is only to test the function APIs
clc
close all
clear all

addpath(genpath("."))

% generate map
true_map = generate_map_from_image("maps/m1.png", false);
% constant collection
cc = constantsCollection();
% define number of robots
cc.n_robots = int8(3);
% define time step
cc.delta_t = 1;
% process noise for robots
cc.process_noise = 1*eye(3);
% total number of time steps
cc.n_time = 50;

% create multi-robot SLAM object
mrs = multiRobotSLAM(true_map, cc);

% run system dynamics
tic
mrs.debug()
for i_time = 1:cc.n_time
    % measure
    measurements = mrs.get_all_measurements();
    
    % all systems evolve
    mrs.all_systems_evolve()
    
    % compute measurement probability | all estiamted poses and maps
    mrs.log_prob_of_measurements(measurements);
end
mrs.debug()
toc

% plot all time history
mrs.plot_all_state_history(true)