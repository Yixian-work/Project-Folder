%%
rng(5);
map = mapMaze(12,5,'MapSize',[10 10],'MapResolution',10); % 

M = occupancyMatrix(map); 
M(40:44,11:22) = 0;

% [J_x,J_y] = meshgrid(12:21,40:46);
world_map = occupancyMap(M,10);  %Initialize World Map from occupancy Matrix
% setOccupancy(world_map,[J_x(:) J_y(:)],zeros(1,length(J_x(:))),"grid");

figure(1)
show(world_map)

%%
sz = size(M);
n = 4; % number of robots
map = occupancyMap(0.5*ones(sz),10); % Initialize Robot Map
frontiers = double.empty(0,2);
% setOccupancy(map,[40,80],1,'grid')
% show(map)
% current_state = grid2world(map, [48, 48; ...
%                                  48, 50; ...
%                                  48, 52; ...
%                                  50, 48; ...
%                                  50, 50; ...
%                                  50, 52; ...
%                                  52, 48; ...
%                                  52, 50; ...
%                                  52, 52]);
current_state = grid2world(map, [48, 48; ...
                                 48, 50; ...
                                 48, 52; ...
                                 50, 48; ...
                                 50, 50; ...
                                 50, 52]);
n = size(current_state,1);
traj  = reshape(current_state',[],n*2);
range = 0.9;
 % number of robots

% MEASUREMENT MAP
n_size = map.GridSize(1);
m_size = map.GridSize(2);
mea_true_grid = zeros(n_size, m_size);
x_1 = 0.169.*m_size;
y_1 = 0.169.*n_size;
k_1 = 150;
x_2 = 0.739.*m_size;
y_2 = 0.169.*n_size;
k_2 = 150;
x_3 = 0.669.*m_size;
y_3 = 0.669.*n_size;
k_3 = 150;

for i = 1:size(mea_true_grid, 1)
    for j = 1:size(mea_true_grid, 2)
        mea_true_grid(i, j) = k_1./((x_1 - j).^2 + (y_1 - i).^2) + k_2./((x_2 - j).^2 + (y_2 - i).^2) + k_3./((x_3 - j).^2 + (y_3 - i).^2);
    end
end
mea_true_grid(logical(checkOccupancy(world_map))) = -1;

mea_evi_grid = zeros(n_size, m_size) - 1;

source_location = [];
mode = ones(1, n); % 1 is explore mode, 0 is gradient following mode
% 
% for i = 1:n
%     current_grid = world2grid(map, current_state);
%     mea_evi_grid(current_grid(i, 1), current_grid(i, 2)) = mea_true_grid(current_grid(i, 1), current_grid(i, 2));
% end

plt = Plotter();

iters = 0;

while 1
        
    
    [next_state,frontiers,map] = state_update(current_state,world_map,map,range,n);
    if isempty(next_state)
        break;
    end
    
    if length(source_location) == 3 
        break;
    end

%     next_grid = local2grid(map, next_state);
    current_grid = local2grid(map, current_state);
    map_plot = checkOccupancy(map);

    for i = 1:n
        current_grid = world2grid(map, current_state);
        mea_evi_grid(current_grid(i, 1), current_grid(i, 2)) = mea_true_grid(current_grid(i, 1), current_grid(i, 2));
        
        dis_to_mode_0 = ones(1, n).*inf;
        for j = 1:n
            if mode(j) == 0
                dis_to_mode_0(j) = norm([current_state(i, 1) - current_state(j, 1), current_state(i, 2) - current_state(j, 2)]);
            end
            if j == i
                dis_to_mode_0(j) = inf;
            end
        end

        if mea_evi_grid(current_grid(i, 1), current_grid(i, 2)) >= 2 && all(dis_to_mode_0 > range*2)
            if isempty(source_location)
                mode(i) = 0;
            else
                dis_to_sources = zeros(1, size(source_location, 1));
                for j = 1:size(source_location, 1)
                    dis_to_sources(j) = norm([current_state(i, 1) - source_location(j, 1), current_state(i, 2) - source_location(j, 2)]);
                end
                if all(dis_to_sources > range*2)
                    mode(i) = 0;
                end
            end
        end

        if mode(i) == 0
            map_plot(current_grid(i, 1), current_grid(i, 2)) = 4;

            if check_source(current_grid(i, 1), current_grid(i, 2), mea_evi_grid)
                if isempty(source_location)
                    source_location = grid2world(map, [current_grid(i, 1), current_grid(i, 2)]);
                else
                    source_location = union(source_location, grid2world(map, [current_grid(i, 1), current_grid(i, 2)]), 'rows');
                end
                mode(i) = 1;
            end
        else
            map_plot(current_grid(i, 1), current_grid(i, 2)) = 2;
        end
        
        if checkOccupancy(map, current_state(i,:)) == 1
            setOccupancy(map, current_state(i, :), 0);
        end

        if mode(i) == 1
            pseudo_map = copy(map);
            occ = checkOccupancy(pseudo_map);
            [unknownRow, unknownCol] = find(occ == -1);
            if ~isempty(unknownRow)
                setOccupancy(pseudo_map,[unknownRow, unknownCol],ones(length(unknownRow),1),"grid");
            end
            planner = plannerAStarGrid(pseudo_map,"GCost","Manhattan");
            backup_planner = plannerAStarGrid(map,"GCost","Manhattan");
            occ = checkOccupancy(map);
        %         next_row_can = [current_grid(i, 1) - 1, current_grid(i, 1) + 1, current_grid(i, 1), current_grid(i, 1)];
        %         next_col_can = [current_grid(i, 2), current_grid(i, 2), current_grid(i, 2) - 1, current_grid(i, 2) +  1];
        %         min_distance = inf;
        %         for j = 1:length(next_row_can)
        %             if next_row_can(j) > 0 && next_row_can(j) <= map.GridSize(1) && next_col_can(j) > 0 && next_col_can(j) <= map.GridSize(2) && occ(next_row_can(j), next_col_can(j)) ~= 1
        %                 distance_to_goal = norm([next_row_can(j) - next_grid(i, 1), next_col_can(j) - next_grid(i, 2)]);
        %                 if distance_to_goal < min_distance
        %                     min_distance = distance_to_goal;
        %                     current_state(i, :) = grid2local(map, [next_row_can(j), next_col_can(j)]);
        %                 end
        %             end
        %         end
            start = world2grid(map,current_state(i,:));
            goal = world2grid(map,next_state(i,:));
            path = plan(planner,start,goal);
            if size(path, 1) < 2
                path = plan(backup_planner,start,goal);
            end
            next_grid = path(2, :);
            next_row_can = [current_grid(i, 1) - 1, current_grid(i, 1) + 1, current_grid(i, 1), current_grid(i, 1)];
            next_col_can = [current_grid(i, 2), current_grid(i, 2), current_grid(i, 2) - 1, current_grid(i, 2) +  1];
            min_distance = inf;
            for j = 1:length(next_row_can)
                if next_row_can(j) > 0 && next_row_can(j) <= map.GridSize(1) && next_col_can(j) > 0 && next_col_can(j) <= map.GridSize(2) && occ(next_row_can(j), next_col_can(j)) ~= 1
                    distance_to_goal = norm([next_row_can(j) - next_grid(1), next_col_can(j) - next_grid(2)]);
                    if distance_to_goal < min_distance
                        min_distance = distance_to_goal;
                        current_state(i, :) = grid2local(map, [next_row_can(j), next_col_can(j)]);
                    end
                end
            end
        else
            current_state(i, :) = new_gradient_step(current_state(i, :), map, mea_evi_grid);
        end
%         % Update the measurement map
%         current_grid = world2grid(map, current_state);
%         mea_evi_grid(current_grid(i, 1), current_grid(i, 2)) = mea_true_grid(current_grid(i, 1), current_grid(i, 2));
%         current_state(i,:) = grid2world(map,path(2,:));
    end

    figure(2);
    for i = 1:size(source_location, 1)
        source_grid = world2grid(map, [source_location(i, 1), source_location(i, 2)]);
        map_plot(source_grid(1), source_grid(2)) = 3;
    end

    traj  = [traj;reshape(current_state',[],n*2)];

%     heatmap(map_plot);
%     cmap = [0.5, 0.5, 0.5; 1, 1, 1; 0, 0, 0; 0.466, 0.674, 0.188; 1, 0, 0; 0 1 0];
%     colormap(cmap);
%     caxis([-1 4])
%     Ax = gca;
%     Ax.XDisplayLabels = nan(size(Ax.XDisplayData));
%     Ax.YDisplayLabels = nan(size(Ax.YDisplayData));
%     grid off
    plt.plot_frontiers(map_plot);
    iters = iters + 1;
end
plt.savegif('9_robot_frontierexp.gif');




