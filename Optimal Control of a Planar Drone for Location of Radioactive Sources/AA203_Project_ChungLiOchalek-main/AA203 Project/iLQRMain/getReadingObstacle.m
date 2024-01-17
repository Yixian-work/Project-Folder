function [S] = getReadingObstacle(y, z, y_ans, z_ans, k, obstacles)
% Takes in current location of the drone (y, z), the location of the source
% (y_ans, z_ans), product of exposure rate constant and activity of the
% source k, and a n * 4 matrix obstacles, in which each row represents a
% circular obstacle with first and second column being the y- and
% z-coordinates of the center of the obstacle, third column being the
% radius of the obstacle, and fourth column being the linear attenuation
% coefficient of the obstacle's material.
% Returns S, the reading of the radiation detector at (y, z).

    n = size(obstacles, 1); % Number of obstacles
    t = nan(n, 1);

    % Turn the straight line from (y, z) to (y_ans, z_ans) into the form z
    % = m*y + c
    m = (z_ans - z)./(y_ans - y);
    c = z - m.*y;
    
    for i = 1:n
        % Extract information from the obstacles matrix
        y_center = obstacles(i, 1);
        z_center = obstacles(i, 2);
        r = obstacles(i, 3);
        
        % Apply quadratic equation to the intersection between the straight
        % line and the circle
        a_quad = 1 + m.^2;
        b_quad = -2.*y_center + 2.*m.*c - 2.*m.*z_center;
        c_quad = y_center.^2 + c.^2 - 2.*c.*z_center + z_center.^2 - r.^2;
        delta = b_quad.^2 - 4.*a_quad.*c_quad; % Discriminant
        if delta <= 0
            t(i) = 0;
        else
            y_1 = (-b_quad + sqrt(delta))./(2.*a_quad);
            z_1 = m.*y_1 + c;
            y_2 = (-b_quad - sqrt(delta))./(2.*a_quad);
            z_2 = m.*y_2 + c;
            direction_actual = atan2(z_ans - z, y_ans - y);
            direction_1 = atan2(z_1 - z, y_1 - y);
            direction_2 = atan2(z_2 - z, y_2 - y);
            % If both roots are between drone and source
            if abs(direction_actual - direction_1) < 0.5 && abs(direction_actual - direction_2) < 0.5
                t(i) = sqrt((z_2 - z_1).^2 + (y_2 - y_1).^2);
            % If only one root is between drone and source (drone is inside the obstacle)
            elseif abs(direction_actual - direction_1) < 0.5
                t(i) = sqrt((z - z_1).^2 + (y - y_1).^2);
            elseif abs(direction_actual - direction_2) < 0.5
                t(i) = sqrt((z - z_2).^2 + (y - y_2).^2);
            % If obstacle is not between drone and source
            else
                t(i) = 0;
            end
        end
    end
    
    % Calculate the final dose rate
    S = k./((y - y_ans).^2 + (z - z_ans).^2);
    for i = 1:n
        mu = obstacles(i, 4);
        S = S.*exp(-mu.*t(i));
    end
end