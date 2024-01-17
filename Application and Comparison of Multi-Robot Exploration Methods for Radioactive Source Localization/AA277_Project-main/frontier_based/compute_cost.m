%% 
function V_f = compute_cost(frontiers,pose,map,iters)
p_occ = occupancyMatrix(map);
p_occ = padarray(p_occ,[1,1],1);
V = Inf(size(p_occ));
ij = world2grid(map,pose)+[1,-1];
V(ij(1),ij(2)) = 0;
for iter = 1:iters
    for i = 2:length(V)-1
        for j = 2:length(V)-1
            Costs = [V(i,j),
                     V(i + 1,j) + p_occ(i + 1,j),
                     V(i - 1,j) + p_occ(i - 1,j),
                     V(i,j + 1) + p_occ(i,j + 1),
                     V(i,j - 1) + p_occ(i,j - 1),
                     V(i + 1,j + 1) + sqrt(2)*p_occ(i + 1,j + 1),
                     V(i + 1,j - 1) + sqrt(2)*p_occ(i + 1,j - 1),
                     V(i - 1,j + 1) + sqrt(2)*p_occ(i + 1,j + 1),
                     V(i - 1,j - 1) + sqrt(2)*p_occ(i + 1,j + 1)];  
            V(i,j) = min(Costs); % cost update
        end
    end
end
V = V(2:end-1,2:end-1);
idx = sub2ind(size(V), frontiers(:,1), frontiers(:,2));% index cost of frontiers
V_f = V(idx);
end
