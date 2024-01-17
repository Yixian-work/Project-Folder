% this script is only to test binary search
clc
close all
clear all

addpath(genpath("."))

angles = linspace(0, 2*pi, 10);

for target = [-0.01, 0, 0.01, 0.5, 3.14, 2*pi-0.01, 2*pi, 2*pi+0.01]
    target_id = binary_search(angles, target);
    fprintf('target is %.3f - found %.3f at id %d\n', target, angles(target_id), target_id)
end