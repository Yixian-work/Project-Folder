% this script is only to test the function APIs

clc
close all
clear all

addpath(genpath("."))

true_map = generate_map_from_image("./maps/m1.png", true);
n_robots = int8(2);
delta_t = 0.1;

% create multi-robot SLAM object
mrs = multiRobotSLAM(true_map, n_robots);
% evolve robot 1one time step
control_input = mrs.get_robot(1).get_control_input([1;1],1,1);
mrs.get_robot(1).system_evolve(delta_t, control_input)
mrs.get_robot(1).get_state();
mrs.get_robot(1).get_map();
mrs.get_true_map();
% measure
[ranges, angles] = mrs.get_robot(1).measure(mrs.get_true_map());

% create multi-robot SLAM solver object
mrs_solver = multiRobotSLAMSolver();
% run EKF algorithm
mrs_solver.solve(mrs, "EKF");

% mrs.network_communicate(0)