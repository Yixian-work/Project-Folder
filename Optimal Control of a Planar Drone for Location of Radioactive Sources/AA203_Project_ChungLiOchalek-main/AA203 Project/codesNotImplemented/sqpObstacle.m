function [s, u] = sqpObstacle(Q, R, Qf, s0, s_goal, N, dt, obstacles)
    %% Setup and Initialize 
    n = size(Q,2);
    m = size(R,2);
    eps = 0.01;
    u_bar = zeros(N,m);
    s_bar = zeros(N+1,m);
    s_bar(1,:) = s0;
    for k = 1:N
        s_bar(k+1,:) = discreteDynamic(s_bar(k,:),u_bar(k,:),dt);
    end
    [s, u] = scp_iteration(Q, R, Qf, s_bar, u_bar, s0, s_goal, N, dt, obstacles);
    %% Iteration 
    round = 0;
    while norm(u-u_bar, Inf) > eps
        fprintf("round: %d, u update: %f\n", round, u);
        round = round + 1;
        s_bar = s;
        u_bar = u;
        [s, u] = scp_iteration(Q, R, Qf, s_bar, u_bar, s0, s_goal, N, dt, obstacles);
    end
end