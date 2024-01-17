function[a] = choose_action(x_r, occ_map, source_map, p_y_s)
    possible_actions = [-1 0 1 0;
                        0 1 0 -1];
    num_actions = size(possible_actions,2);
    mutual_info = zeros(1,num_actions);
    prob_y = zeros(1,num_actions);
    prob_c = zeros(1,num_actions);
    for i = 1:size(possible_actions,2)
        x_next = x_r+possible_actions(:,i);
        if x_next(1) <occ_map.XWorldLimits(1) || x_next(1) >occ_map.XWorldLimits(2) ||...
                x_next(2) <occ_map.YWorldLimits(1) || x_next(2) >occ_map.YWorldLimits(2)
            mutual_info(i) = -1;
        elseif getOccupancy(occ_map, x_next' ) == 1
            mutual_info(i) = -1;
            prob_c(i) = getOccupancy(source_map,x_next');
        else
            mutual_info(i) = calculate_MI(x_r + possible_actions(:,i), source_map, p_y_s);
        end
        prior = calculate_prior(x_next,source_map);
        prob_y(i) = (prior*p_y_s');
    end
    mutual_info_scaled = mutual_info/max(mutual_info);
    rand_val = rand(size(mutual_info));
    utility = mutual_info_scaled ...
                + 0.4*prob_y ...
                + 0.2*rand_val;
    [~,idx] = max(utility);
    a = possible_actions(:,idx);
end
