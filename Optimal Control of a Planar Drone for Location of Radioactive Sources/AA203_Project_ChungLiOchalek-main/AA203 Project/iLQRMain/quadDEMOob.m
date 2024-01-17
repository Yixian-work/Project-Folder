%% DEMO: EXAMPLE 2 - Obstacles
%% Define a initial condition and a goal state, define a time
% duration, cost matrices, find an ultimate trajectory.
n = 6;  % state dimension
m = 2;  % control dimension
Q = diag([10, 10, 10, 2, 2, 2]);  % state cost matrix
R = 0.01*eye(m); % control cost matrix
Qf = 100*eye(n); % terminal state cost matrix
Qob = 100*eye(n); % Terminal state cost 
s0 = [0, 0, 0, 0, 0, 0]; % initial state, demo defined
s_goal = [5, 5, 0, 0, 0, 0]; % goal state, demo defined
T = 1;   % simulation time
dt = 0.1;  % sampling time
%% Define an circular obstacle radius.
radius_ob = 1.3;
ctr = [2,5, 2,5];
%% Running iLQR to get control policies
fprintf('Computing iLQR solution');
t = 0:dt:T;
N = size(t,2)-1;
[l, L, s_bar, u_bar] = iLQR(Q, R, Qf, s0, s_goal, N, dt);
fprintf('Done computing');
%% Apply control policies to continuous-time
s = zeros(N+1, n);
u = zeros(N, m);
s(1,:) = s0;
for k = 1:N
    Lk = reshape(L(k,:,:), [m, n]);
    u(k,:) = u_bar(k,:)' + Lk*(s(k,:)-s_bar(k,:))' + l(k,:)';
    [ts, sp] = ode45(@(t,x) droneDynamicODE(x,u(k,:)), [t(k) t(k+1)], s(k,:));
    s(k+1,:) = sp(end,:);
end    
%% Plotting
plot(s(:,1), s(:,2))
title('A demo of the trajectory of the drone (10 seconds simulation)')
xlabel('x coordinate')
ylabel('y coordinate')