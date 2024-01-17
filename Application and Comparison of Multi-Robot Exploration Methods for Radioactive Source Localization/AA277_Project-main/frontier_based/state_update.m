function [next_state,frontiers,map] = state_update(current_state,world_map,map,range,n)
    
    for i = 1:n
        [frontiers,map] = Scan(current_state(i,:),world_map,map,range); % returns frontiers in map indexes
    end
    occ = checkOccupancy(map,frontiers,"grid");
    occ_idx = find(occ == 1);
    if ~isempty(occ_idx)
        frontiers(occ_idx,:) = [];
    end
    if size(frontiers, 1) < n
        next_state = [];
        frontiers = [];
        map = [];
    else
        V_f = zeros(size(frontiers,1),n);
        for i = 1:n
            V_f(:,i) = compute_cost(frontiers,current_state(i,:),map,100); % Compute cost to go to each frontier from every robot position
        end
        
        % returns cost to go for each frontier
        U_f = ones(size(V_f,1),1); %initialize utility
        
        next_state = [];
        for i = 1:n
            [C,argmax] = max(U_f-V_f(:,i)); % find argmax
            target_frontier = grid2world(map,frontiers(argmax,:)); %return target frontier in world coordinates
            [U_f,frontiers,V_f] = reduce_utility(V_f,U_f,map,argmax,target_frontier,frontiers,range); %return updated utility & updated frontiers
            next_state = [next_state;target_frontier];
%             if mode(i) == 0
%                 next_state(i, :) = new_gradient_step(current_state, map, mea_evi_grid);
%             end
        end
    end
end