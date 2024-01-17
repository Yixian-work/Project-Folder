%% Set up true grid
rng(5);
n = 100;
m = 100;
mea_true_grid = zeros(n, m);
y_1 = 0.669.*n;
x_1 = 0.329.*m;
k_1 = 150;
y_2 = 0.259.*n;
x_2 = 0.839.*m;
k_2 = 150;
y_3 = 0.219.*n;
x_3 = 0.169.*m;
k_3 = 150;

for i = 1:size(mea_true_grid, 1)
    for j = 1:size(mea_true_grid, 2)
        mea_true_grid(i, j) = k_1./((x_1 - j).^2 + (y_1 - i).^2) + k_2./((x_2 - j).^2 + (y_2 - i).^2) + k_3./((x_3 - j).^2 + (y_3 - i).^2);
    end
end
% mea_true_grid(:, 1) = -1;
% mea_true_grid(1, :) = -1;
% mea_true_grid(end, :) = -1;
% mea_true_grid(:, end) = -1;
% mea_true_grid(50:60, 10:30) = -1;
% mea_true_grid(50:80, 50:60) = -1;

map = mapMaze(12,5,'MapSize',[10 10],'MapResolution',10); % 
M = occupancyMatrix(map); 
world_map = occupancyMap(M,10);  %Initialize World Map from occupancy Matrix
occ_evi_grid = checkOccupancy(world_map);
mea_true_grid(logical(occ_evi_grid)) = -1;

%% Set up occupuncy grid
% occ_evi_grid = mea_true_grid == -1;

%% Experiment
% Initialize variables
y_0 = [92, 89, 20];
x_0 = [54, 12, 8];
% y_0 = [68, 89, 72];
% x_0 = [24, 12, 42];
mea_evi_grid = zeros(n, m) - 1;
for i = 1:length(x_0)
    mea_evi_grid(y_0(i), x_0(i)) = mea_true_grid(y_0(i), x_0(i));
end
cat_grid = make_cat_grid(mea_evi_grid);
cat = [];
for i = 1:length(x_0)
    cat = [cat, cat_grid(y_0(i), x_0(i))];
end
y_history = [y_0'];
x_history = [x_0'];
is_source_found = zeros(length(x_0));
while ~all(is_source_found)
    y_history = [y_history, zeros(length(x_0), 1)];
    x_history = [x_history, zeros(length(x_0), 1)];
    for i = 1:length(x_0)
        cur_y = y_history(i, end - 1);
        cur_x = x_history(i, end - 1);
        if check_source(cur_y, cur_x, mea_evi_grid)
            y_history(i, end) = cur_y;
            x_history(i, end) = cur_x;
            is_source_found(i) = 1;
        else
            [next_y, next_x] = gradient_step(mea_evi_grid, occ_evi_grid, cur_y, cur_x, cat_grid, cat(i));
            y_history(i, end) = next_y;
            x_history(i, end) = next_x;
            mea_evi_grid(next_y, next_x) = mea_true_grid(next_y, next_x);
            cat_grid = make_cat_grid(mea_evi_grid);
            cat(i) = cat_grid(y_0(i), x_0(i));
        end
    end
    animate_cat_grid = cat_grid;
    animate_cat_grid(mea_true_grid == -1) = 0;
    animate_cat_grid(round(y_3), round(x_3)) = 0.5;
    animate_cat_grid(round(y_2), round(x_2)) = 0.5;
    animate_cat_grid(round(y_1), round(x_1)) = 0.5;
    heatmap(animate_cat_grid);
    pause(.1);
end