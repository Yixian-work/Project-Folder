function[A, B] = linearizeDrone(s, u, dt)
%% Linearization function of the drone dynamics.
    %% Jacobian Symbolic
    a = sym('a',[1 6]);
    b = sym('b',[1 2]);        
    Ak = jacobian(discreteDynamic(a,b,dt),a);
    Bk = jacobian(discreteDynamic(a,b,dt),b);
    %% Plug in values and formulate A and B
    N = size(s, 1);
    for k = 1:N
        A(k,:,:) = vpa(subs(Ak, [a, b], [s(k,:), u(k,:)]));
        B(k,:,:) = vpa(subs(Bk, [a, b], [s(k,:), u(k,:)]));
    end
end