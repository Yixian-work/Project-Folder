function[posterior] = calculate_posterior(x_r, y, source_map, p_y_s)
    % x_r: current robot position
    % y: current measurement (0 or 1)
    % source_map: source_map containing prior probability of state
    % distribution
    % p_y_s: sensor model (1*2^9)
    if y == 0
        p_y_s = 1.- p_y_s;
    end
    %posterior = zeros(1,2^9);
    p_s = calculate_prior(x_r, source_map); %(1*2^9)
    posterior = p_s.*p_y_s;
    posterior = posterior/sum(posterior);
end
