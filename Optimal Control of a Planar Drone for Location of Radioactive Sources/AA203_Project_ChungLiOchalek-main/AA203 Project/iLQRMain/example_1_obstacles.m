%% Example code
%% User-Defined variables
% Starting location of the drone
y_0 = 0;
z_0 = 0;

% Distance between each time-step
d = 1; % Just for example; In reality, steps should be characterized by time

% RNG parameters to generate where the radioactive source is
rng(0); % RNG seed
r_min = 5; % Minimum radius of goal from starting location
r_max = 10; % Maximum radius of goal from starting location

% Simulation parameters
k = 3; % Strength of the source
epsilon = 0.1; % Desired accuracy of the estimation
n = 100; % Maximum allowed iterations

% Obstacle is a circle with center at (10, 0) with radius 2 and mu = 1
obstacles = [2, 4, 0.5, 0.3; 4, 2, 0.5, 0.3];

%% Automate from here
% Generate the location of the goal
% z_ans = -inf;
% while z_ans < 0 % Regenerate goal if it is below ground
%     % Goal in cylindrical coordinates
%     r_ans = r_min + rand.*(r_max - r_min);
%     theta_ans = 2.*pi.*rand;
%     % Convert to Cartesian coordinates
%     y_ans = y_0 + r_ans.*cos(theta_ans);
%     z_ans = z_0 + r_ans.*sin(theta_ans);
% end
y_ans = 5;
z_ans = 5;

% Move the drone to get some initial data
% This is unnecessary but it reduces errors (see Chung et al. 2021)
% y_history = zeros(1, 4).*y_0;
% z_history = nan(size(y_history));
% S_history = nan(size(y_history));
% for i = 1:length(y_history)
%     z_history(i) = z_0 + d.*(i - 1); % Fly the drone upwards
%     S_history(i) = getReadingObstacle(y_history(i), z_history(i), y_ans, z_ans, k, obstacles);
% end
y_history = y_0;
z_history = z_0;
S_history = getReadingObstacle(y_history, z_history, y_ans, z_ans, k, obstacles);

% Generate trajectory of the drone
delta = inf;
y_guess_history = [];
z_guess_history = [];

for i = 1:n
    % Get next direction and estimated source location
    [phi, y_guess, z_guess] = guessSourceObstacle(y_history, z_history, S_history, obstacles);
    y_guess_history = [y_guess_history, y_guess];
    z_guess_history = [z_guess_history, z_guess];
    
    % Computes 2-norm of differences between last two guesses
    if i > 1
        delta = norm([y_guess_history(end) - y_guess_history(end - 1), z_guess_history(end) - z_guess_history(end - 1)]);
    end
    
    % Terminates if two successive guesses are close enough AND the guess
    % is within reach
    remainingDistance = norm([y_guess - y_history(end), z_guess - z_history(end)]);
    if delta < epsilon && remainingDistance <= d
        y_history = [y_history, y_guess];
        z_history = [z_history, z_guess];
        S_history = [S_history, getReadingObstacle(y_guess, z_guess, y_ans, z_ans, k, obstacles)];
        break
        
    % Continue otherwise
    else
        y_next = y_history(end) + d.*cos(phi);
        z_next = z_history(end) + d.*sin(phi);
        y_history = [y_history, y_next];
        z_history = [z_history, z_next];
        S_history = [S_history, getReadingObstacle(y_next, z_next, y_ans, z_ans, k, obstacles)];
    end
    disp(i)
end

%% Plotting
figure();
hold on;
plot(y_ans, z_ans, '*k'); % Answer
plot(y_history, z_history, '-ok'); % Trajectory
xlim([y_0 - r_max, y_0 + r_max]);
% ylim([max(0, z_0 - r_max), z_0 + r_max]);
ylim([-6, z_0 + r_max]);
xlabel('y (m)');
ylabel('z (m)');

% Plot obstacle
for i = 1:size(obstacles, 1)
    y = linspace(obstacles(i, 1) - obstacles(i, 3), obstacles(i, 1) + obstacles(i, 3));
    plot(y, sqrt(obstacles(i, 3).^2 - (y - obstacles(i, 1)).^2) + obstacles(i, 2), '-k');
    plot(y, -sqrt(obstacles(i, 3).^2 - (y - obstacles(i, 1)).^2) + obstacles(i, 2), '-k');
end
legend('Source', 'Drone', '', '');
hold off;

%% Helper functions
function S = getReading(y, z, y_ans, z_ans, k)
% Obtain the radiation detector reading S from reading location (y, z),
% source location (y_ans, z_ans), and source strength k
    S = k./((y - y_ans).^2 + (z - z_ans).^2);
end