function[p_s] = calculate_prior(x_r, source_map)
    % get a source probability info for adjacent grid space
    sensor_params;

    prob_adj = zeros(1,sensor_num_cells);
    for i = 1:sensor_num_cells
        x_next = x_r+sensor_mask(:,i);
        if x_next(1) <source_map.XWorldLimits(1) || x_next(1) >source_map.XWorldLimits(2) ||...
                x_next(2) <source_map.YWorldLimits(1) || x_next(2) >source_map.YWorldLimits(2)
            prob_adj(i) = 0;
        else
            prob_adj(i) = getOccupancy(source_map, x_next' );
        end
    end
    s_complement = sensor_states==0;
    p_s = prod(abs(s_complement-prob_adj)');
end
