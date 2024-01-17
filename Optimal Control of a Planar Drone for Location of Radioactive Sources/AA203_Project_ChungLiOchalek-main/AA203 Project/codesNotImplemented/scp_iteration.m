function [s,u] = scp_iteration(Q, R, Qf, s_bar, u_bar, s0, s_goal, N, dt, obstacles)
    n = size(Q,1);
    m = size(R,1);
    eps = 0.01;
    max_iters = 100;
    n_obstacle = size(obstacles, 1); % Number of obstacles
    rho = 0.5;
    cvx_begin
        variable x(N+1,n);
        variable u(N,m);
        cost = (1/2)*quad_form(s(N+1,:)-s_goal, Qf);
        for i = 1:N
            cost = cost + (1/2)*quad_form(s(i,:) - s_goal, Q) + (1/2)*quad_form(u(i,:), R);
        end
        minimize cost;
        subject to
            s(1,:) == s0;
        for i = 1: N
            A, B, c = linearize(f, s_bar(i,:), u_bar[i])
        Constraints += [s[i+1,:] == A @ (s[i,:] - s_bar[i,:]) + B @ (u[i,:] - u_bar[i,:]) + c]
        Constraints += [cvx.norm(s[i,:] - s_bar[i,:], 'inf') <= rho]
        Constraints += [cvx.norm(u[i,:] - u_bar[i,:], "inf") <= rho]
        Constraints += [u[i,:] <= uUB, u[i,:] >= uLB]
        end
    cvx_end
end