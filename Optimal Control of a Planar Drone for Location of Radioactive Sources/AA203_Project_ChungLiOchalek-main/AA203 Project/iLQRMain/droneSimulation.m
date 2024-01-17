%% This file is the drone simulation for open space. (Done.)
%% Define source location and k 
source_truth = [5, 5]';
y_ans = source_truth(1);
z_ans = source_truth(2);
d = 1;
epsilon = 0.01;
%% Get some measurements from random places and establish first goal guess
cur_pose = [0, 0, 0, 0, 0, 0]';
y_history = [0];
z_history = [0];
y_trajectory = [0];
z_trajectory = [0];
F_data = [];
M_data = [];
S_history = getReading(y_history, z_history, y_ans, z_ans, 3);
y_guess_history = [];
z_guess_history = [];
%% Iteratate simulation
for i = 1:40
    % Get next direction and estimated source location
    [phi, y_guess, z_guess] = guessSource(y_history, z_history, S_history);
    y_guess_history = [y_guess_history, y_guess];
    z_guess_history = [z_guess_history, z_guess];
    %% Computes 2-norm of differences between last two guesses
    if i > 1
        delta = norm([y_guess_history(end) - y_guess_history(end - 1), z_guess_history(end) - z_guess_history(end - 1)]);
    end
    %% Terminate condition
    remainingDistance = norm([y_guess - y_history(end), z_guess - z_history(end)]);
    if i > 1 && delta < epsilon && remainingDistance < d
        y_history = [y_history, y_guess];
        z_history = [z_history, z_guess];
        S_history = [S_history, getReading(y_guess, z_guess, y_ans, z_ans, 3)];
        break
    end
    %% ILQR parameters (Tunable)
    n = 6;  % state dimension
    m = 2;  % control dimension
    Q = diag([10, 10, 10, 2, 2, 2]);  % state cost matrix
    R = 0.1*eye(m); % control cost matrix
    Qf = 100*eye(n); % terminal state cost matrix
    s0 = cur_pose; % cur state as initial state
    s_goal = [y_guess, z_guess, 0, 0, 0, 0]; % guess source as goal state
    T = 5;   % simulation time (IMPORTANT) Bigger than 1!!
    dt = 0.2;  % sampling time
    %% Running iLQR to get control policies
    fprintf('Computing iLQR solution');
    t = 0:dt:T;
    N = size(t,2)-1;
    [l, L, s_bar, u_bar] = iLQR(Q, R, Qf, s0, s_goal, N, dt);
    fprintf('Done computing');
    %% Apply control policies to continuous-time with 1 seconds and update current position
    tm = 0:dt:1;
    Nm = size(tm,2)-1;
    s = zeros(Nm+1, n);
    u = zeros(Nm, m);
    s(1,:) = s0;
    for k = 1:Nm
        Lk = reshape(L(k,:,:), [m, n]);
        u(k,:) = u_bar(k,:)' + Lk*(s(k,:)-s_bar(k,:))' + l(k,:)';
        [ts, sp] = ode45(@(t,x) droneDynamicODE(x,u(k,:)), [t(k) t(k+1)], s(k,:));
        s(k+1,:) = sp(end,:);
    end
    %% Update the history 
    y_trajectory = [y_trajectory, s(2:end,1)'];
    z_trajectory = [z_trajectory, s(2:end,2)'];
    y_history = [y_history, s(end,1)];
    z_history = [z_history, s(end,2)];
    S_history = [S_history, getReading(s(end,1), s(end,2), y_ans, z_ans, 3)];
    F_data = [F_data, u(:,1)'];
    M_data = [M_data, u(:,2)'];
    cur_pose = s(end,:);
end