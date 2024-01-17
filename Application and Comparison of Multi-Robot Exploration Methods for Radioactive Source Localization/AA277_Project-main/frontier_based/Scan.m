%% Occupancy Grid Update
function [frontiers,map] = Scan(pose,world_map,map,range)
    angles = linspace(0,2*pi,360);
    emptyPts = double.empty(0,2);
    wallPts = double.empty(0,2);
%     occ = checkOccupancy(map);
%     [emptyRow, emptyCol] = find(occ == 0);
%     if ~isempty(emptyRow)
%         emptyPts = [emptyRow, emptyCol];
%     else
%         emptyPts = double.empty(0,2);
%     end
%     [wallRow, wallCol] = find(occ == 1);
%     if ~isempty(emptyRow)
%         wallPts = [wallRow, wallCol];
%     else
%         wallPts = double.empty(0,2);
%     end
    
    for i = 1:length(angles)
        intersectionpts = rayIntersection(world_map,[pose,0],angles(i),range); % detect obstacles along a ray (returns NaN if no obstacle)
        if isnan(intersectionpts)
            [endpoints, midpoints] = raycast(world_map,[pose,0],range,angles(i)); % raycast based on range if no intersect
            emptyPts = union(emptyPts,midpoints,'rows');  %add new frontiers 
            emptyPts = union(emptyPts, endpoints, 'rows');
        else
            [endpoints,midpoints] = raycast(map,pose,intersectionpts); % raycast base on pose & intersection point
            emptyPts = union(emptyPts, midpoints, 'rows');
            wallPts = union(wallPts,endpoints,"rows"); % Add wallPts, round to nearest .05
        end
    end
    occ = checkOccupancy(map,emptyPts,"grid");
    occ_idx = find(occ == 1);
    if ~isempty(occ_idx)
        emptyPts(occ_idx, :) = [];
    end
    setOccupancy(map,emptyPts,zeros(length(emptyPts),1),'grid');
    if ~isempty(wallPts)
        setOccupancy(map,wallPts,ones(length(wallPts),1), 'grid');
    end
    frontiers = findFrontiers(map);
    
% 
%     %if you remove this, all frontiers for a given timestep will be detected,
%     %but I need frontiers to update with movement by removing frontiers that
%     %are also in midPts
%     ex_frontiers = ismember(frontiers,midPts,"rows"); % Remove old frontiers that have been explored
%     ex_idx = find(ex_frontiers == 1); 
%     frontiers(ex_idx,:) = [];
% 
%     obs_frontiers = ismember(frontiers,world2grid(map,wallPts),"rows"); % Remove wallPts from frontiers
%     obs_idx = find(obs_frontiers == 1); 
%     frontiers(obs_idx,:) = [];  
% 
%     occ = checkOccupancy(map,frontiers,"grid");
%     occ_idx = find(occ == 1);
%     if ~isempty(occ_idx)
%         frontiers(occ_idx,:) = [];
%     end
        
%     setOccupancy(map,midPts,zeros(length(midPts),1),'grid');   % set occupancy of mid points to zero
   
%     setOccupancy(map,wallPts,ones(length(wallPts),1)); % set occupancy of wallpoints to 1
%     figure(2)
%     show(map)
%     
%     figure(3)
%     frontier_viz = copy(map);
%     occ = checkOccupancy(frontier_viz);
%     [wallRow, wallCol] = find(occ == 1);
%     if ~isempty(wallRow)
%         setOccupancy(frontier_viz,[wallRow, wallCol],zeros(length(wallRow),1),"grid");
%     end
%     
%     setOccupancy(frontier_viz,frontiers,ones(size(frontiers, 1),1),"grid");
%     setOccupancy(frontier_viz, pose, 1);
%     show(frontier_viz)
end

function [frontiers] = findFrontiers(map)
    occ = checkOccupancy(map);
    [emptyRow, emptyCol] = find(occ == 0);
    frontiers = [];
    for i = 1:length(emptyRow)
        if any([isValid(emptyRow(i) - 1, emptyCol(i), occ) && occ(emptyRow(i) - 1, emptyCol(i)) == -1, isValid(emptyRow(i) + 1, emptyCol(i), occ) && occ(emptyRow(i) + 1, emptyCol(i)) == -1, isValid(emptyRow(i), emptyCol(i) - 1, occ) && occ(emptyRow(i), emptyCol(i) - 1) == -1, isValid(emptyRow(i), emptyCol(i) + 1, occ) && occ(emptyRow(i), emptyCol(i) +  1) == -1])
            frontiers = [frontiers; emptyRow(i), emptyCol(i)];
        end
    end
end

function [validity] = isValid(row, col, mat)
    if row > 0 && col > 0 && row <= size(mat, 1) && col <= size(mat, 2)
        validity = 1;
    else
        validity = 0;
    end
end
