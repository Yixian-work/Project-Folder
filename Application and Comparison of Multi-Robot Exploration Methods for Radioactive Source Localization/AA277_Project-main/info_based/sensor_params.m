% global file to change sensor params

% radius of sensor range
% X_sense = {x+s | norm(s)<=sensor_rnage}
sensor_range = 2;
% positions on the grid relative to the sensor that the sensor reaches
sensor_mask = get_sensor_mask(sensor_range);
sensor_num_cells = size(sensor_mask,2);
sensor_num_states = 2^sensor_num_cells;
% probability of y=1 if a source is within range
sensor_prob_1 = .95;
% probability of y=1 if a source is NOT within range
sensor_prob_0 = 0.05;
% saves instead of calculating each time
load('sensor_states.mat')

create_sensor_states = @() gen_sensor_states(sensor_num_states);

function sensor_states = gen_sensor_states(sensor_num_states)
    sensor_states = (str2double(num2cell(dec2bin(0:sensor_num_states-1))) );
    save('sensor_states','sensor_states');
end

function sensor_mask = get_sensor_mask(r)
    [X,Y] = meshgrid(-r:r,-r:r);
    grid = cat(3,X,Y);
    mask = vecnorm(grid,3,3)<=r;
    sensor_mask = [X(mask),Y(mask)]';
end
