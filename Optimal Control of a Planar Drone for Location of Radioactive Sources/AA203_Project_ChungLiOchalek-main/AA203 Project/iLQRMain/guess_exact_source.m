%% Assume we know everything about the obstacle, given reading, current state, return the contour of radiation received.
function [phi, y_guess, z_guess] = guess_exact_source(y_history, z_history, S_history, obstacles)
% Return a contour points w.r.t to current state, which corresponds to the
% reading received.
% y_history, z_history: history of drone position
% S: Strength of radiation (measurement)
% obstacle: Information of obstacles. first and second column being the y- and
% z-coordinates of the center of the obstacle, third column being the
% radius of the obstacle, and fourth column being the linear attenuation
% coefficient of the obstacle's material.
    
end