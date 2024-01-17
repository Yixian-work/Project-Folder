function[l, L, s_bar, u_bar] = iLQRObstacle(Q, R, Qf, s0, s_goal, N, dt, obstacles, mu)
    %% Get state and control dimension, specify max iteration 
    n = size(Q,1);
    m = size(R,1);
    eps = 0.001;
    max_iters = 100;
    n_obstacle = size(obstacles, 1); % Number of obstacles
    %% Initialize control law terms 'L' and 'l'
    L = zeros(N,m,n);
    l = zeros(N,m);
    %% Initialize `u`, `u_bar`, `s`, and `s_bar` with a forward pass
    u_bar = zeros(N, m);
    s_bar = zeros(N + 1, n);
    s_bar(1,:) = s0;
    for k = 1:N
        s_bar(k+1,:) = discreteDynamic(s_bar(k,:), u_bar(k,:), dt);
    end
    u = u_bar;
    s = s_bar;
    %% iLQR Loop
    converged = false;
    for i = 1:max_iters
        fprintf("iterations %f", i)
        %% Linearlize to get Jacobian
        [A, B] = linearizeDrone(s_bar(1:end-1,:), u_bar(:,:), dt);
        %% Backward Pass
        v = (1/2)*(s_bar(N+1,:) * Qf * s_bar(N+1,:)') - s_goal * Qf * s_bar(N+1,:)' + (1/2)*(s_goal * Qf * s_goal');
        vb = Qf' * (s_bar(N+1,:)' - s_goal');
        V = Qf;
        for k = N:-1:1
            %% Calculate Q values
            Ak = reshape(A(k,:,:),[n,n]);
            Bk = reshape(B(k,:,:),[n,m]);
            Qk = (1/2)*(s_bar(k,:) * Q * s_bar(k,:)') - s_goal * Q * s_bar(k,:)'...
                 + (1/2)*(s_goal * Q * s_goal') + (1/2)*(u_bar(k,:) * R * u_bar(k,:)') + v;
            Qxk = Q' * (s_bar(k,:) - s_goal)' + Ak' * vb;
            Quk = R' * u_bar(k,:)' + Bk' * vb;
            Qxxk = Q + Ak' * V * Ak;
            Quuk = R + Bk' * V * Bk;
            Quxk = Bk' * V * Ak;
            %% Account for obstacles in cost function
            for j = 1:n_obstacle
                r = obstacles(j, 3);
                s_center = [obstacles(j, 1), obstacles(j, 2), 0, 0, 0, 0];
                K = [1 0 0 0 0 0; 0 1 0 0 0 0; zeros(4, 6)];
                isCollide = r^2 - (s_bar(k, :) - s_center)*K*(s_bar(k, :) - s_center)';
                if isCollide > 0
                    Qk = Qk + 0.5*mu*isCollide;
                    Qxk = Qxk - mu*(s_bar(k, :)*K - s_center*K)';
                    Qxxk = Qxxk - mu*K;
                end
            end
            %% Update s_bar, u_bar, L, l
            l(k,:) = -inv(Quuk) * Quk;
            Lk = -inv(Quuk) * Quxk;
            L(k,:,:) = Lk;
            %% Update value function term
            v = Qk - (1/2)*(l(k,:) * Quuk * l(k,:)');
            vb = Qxk - Lk' * Quuk * l(k,:)';
            V = Qxxk - Lk' * Quuk * Lk;
        end
        %% Forward pass (Update new inputs and new states)
        for k = 1:N
            Lk = reshape(L(k,:,:),[m,n]);
            u(k,:) = u_bar(k,:)' + Lk * (s(k,:) - s_bar(k,:))' + l(k,:)';
            s(k+1,:) = discreteDynamic(s(k,:), u(k,:), dt);
        end
        %% Stopping Criteria and Update
        if max(abs(u - u_bar)) < eps
            converged = true;
            break;
        else
            u_bar = u;
            s_bar = s;
        end
    end
    %% Print out error message!
    if converged == false
        fprintf('iLQR did not converge!');
    end
end