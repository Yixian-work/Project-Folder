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
epsilon = 0.01; % Desired accuracy of the estimation
n = 40; % Maximum allowed iterations

%% Automate from here
% Generate the location of the goal
z_ans = -inf;
while z_ans < 0 % Regenerate goal if it is below ground
    % Goal in cylindrical coordinates
    r_ans = r_min + rand.*(r_max - r_min);
    theta_ans = 2.*pi.*rand;
    % Convert to Cartesian coordinates
    y_ans = y_0 + r_ans.*cos(theta_ans);
    z_ans = z_0 + r_ans.*sin(theta_ans);
end
y_ans = 5;
z_ans = 5;
% Move the drone to get some initial data
% This is unnecessary but it reduces errors (see Chung et al. 2021)
y_history = ones(1, 3).*y_0;
z_history = [z_0, z_0 + d, z_0 + 2.*d]; % Fly the drone upwards
S_history = getReading(y_history, z_history, y_ans, z_ans, k);

y_history = [0, 2, 3];
z_history = [0, 2, 3];
S_history = getReading(y_history, z_history, y_ans, z_ans, k);
% Generate trajectory of the drone
delta = inf;
y_guess_history = [];
z_guess_history = [];

for i = 1:n
    % Get next direction and estimated source location
    [phi, y_guess, z_guess] = guessSource(y_history, z_history, S_history);
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
        S_history = [S_history, getReading(y_guess, z_guess, y_ans, z_ans, k)];
        break
        
    % Continue otherwise
    else
        y_next = y_history(end) + d.*cos(phi);
        z_next = z_history(end) + d.*sin(phi);
        y_history = [y_history, y_next];
        z_history = [z_history, z_next];
        S_history = [S_history, getReading(y_next, z_next, y_ans, z_ans, k)];
    end
end

%% Plotting
figure();
hold on;
plot(y_ans, z_ans, '*k'); % Answer
plot(y_history, z_history, '-ok'); % Trajectory
xlim([y_0 - r_max, y_0 + r_max]);
ylim([max(0, z_0 - r_max), z_0 + r_max]);
legend('Source', 'Drone');
xlabel('y (m)');
ylabel('z (m)');
hold off;

%% Helper functions
function S = getReading(y, z, y_ans, z_ans, k)
% Obtain the radiation detector reading S from reading location (y, z),
% source location (y_ans, z_ans), and source strength k
    S = k./((y - y_ans).^2 + (z - z_ans).^2);
end