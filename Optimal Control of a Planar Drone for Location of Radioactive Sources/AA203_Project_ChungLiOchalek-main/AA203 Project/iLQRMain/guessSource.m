function [phi, y_guess, z_guess] = guessSource(y_history, z_history, S_history)
% Returns estimated source location (y_guess, z_guess) and phi, the
% suggested direction the drone should fly in. Takes input history of the
% drone's path (y_history, z_history), S_history, the corresponding
% detector readings at the pathpoints.

    % Equation determining the source's dynamics
    Eq = @(b, yz) b(1)./((yz(:, 1) - b(2)).^2 + (yz(:, 2) - b(3)).^2);
    
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