function [phi, y_guess, z_guess] = guessSourceObstacle(y_history, z_history, S_history, obstacles)
    % Equation determining the source's dynamics
    Eq = @(b, yz) cost_function(yz(:, 1), yz(:, 2), b(2), b(3), b(1), obstacles);
    
    % Define variables
    yz_history = [y_history', z_history'];
    beta_0 = [1, 1, 1]'; % Where to start the guess
    
    % Apply Levenberg-Marquardt to solve the least squares problem
    beta = nlinfit(yz_history, S_history', Eq, beta_0);
    
    % Returns result
    y_guess = beta(2);
    z_guess = beta(3);
    phi = atan2(z_guess - z_history(end), y_guess - y_history(end));
end