function[y] = generate_measurement_single(x_r, source)
    sensor_params;
    % Single robot measurement generation.
    dists = vecnorm(x_r - source);
    mindist = min(dists);
    prob = rand();
    if mindist <=sensor_range
        y = prob<=sensor_prob_1;
    else
        y = prob<=sensor_prob_0;
    end
    %if mindist == 0
    %    y = (eval <= 0.99);
    %elseif mindist == 1
    %    y = (eval <= 0.05);
    %elseif mindist == sqrt(2)
    %    y = (eval <= 0.025);
    %else
    %    y = 0;
    %end
end
