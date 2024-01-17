function[source_map] = update_map(x_r, posterior, source_map)
    sensor_params;
    
    p_si = sensor_states'*posterior';
    %p_si(p_si<.01) = .02*(rand(size(p_si(p_si<.01)))+1);
    for j = 1:sensor_num_cells
        x_next = x_r+sensor_mask(:,j);
        if x_next(1) <source_map.XWorldLimits(1) || x_next(1) >source_map.XWorldLimits(2) ||...
                x_next(2) <source_map.YWorldLimits(1) || x_next(2) >source_map.YWorldLimits(2)
            continue
        else
            setOccupancy(source_map, (x_r + sensor_mask(:,j))', p_si(j));
    end
end
