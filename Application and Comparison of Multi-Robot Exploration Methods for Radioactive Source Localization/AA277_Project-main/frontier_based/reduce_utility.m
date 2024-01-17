%%
function [U,available_frontiers,V] = reduce_utility(V,U,map,argmax,target_frontier,available_frontiers,range)
    beta = 2;
    U(argmax) = [];
    V(argmax,:) = [];
    available_frontiers(argmax,:) = [];% removes target_frontier from available frontiers
    U = U - beta * P(target_frontier,available_frontiers,map,range); %Utility Reduction
end

%%
function p = P(target_frontier, available_frontiers,map,range)
    r = grid2world(map,available_frontiers) - target_frontier; % t_i - t'
    d = vecnorm(r,2,2);
    p = 1 - d/range;
    for i = 1:length(available_frontiers)
%         intersectionpt =rayIntersection(map,[target_frontier, 0] ,atan2(r(i,2),r(i,1)),d(i),0.7)
        [endPts,midPts] =raycast(map,target_frontier,r(i,:)+target_frontier); %cast ray between t_i & t'
        occ = checkOccupancy(map,[endPts;midPts],"grid");
        if any(occ == 1)
            p(i) = 0; %set P(t - t') = 0 if ray intersects wall
        end
    end
    mask = d <= range;
    p = p.*mask;
end