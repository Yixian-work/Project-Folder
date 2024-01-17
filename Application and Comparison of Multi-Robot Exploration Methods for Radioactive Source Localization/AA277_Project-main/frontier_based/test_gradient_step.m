%% Set up true grid
n = 100;
m = 100;
mea_true_grid = zeros(n, m);
y_1 = 0.148.*n;
x_1 = 0.258.*m;
k_1 = 100;
y_2 = 0.699.*n;
x_2 = 0.788.*m;
k_2 = 150;
y_3 = 0.387.*n;
x_3 = 0.522.*m;
k_3 = 150;

for i = 1:size(mea_true_grid, 1)
    for j = 1:size(mea_true_grid, 2)
        mea_true_grid(i, j) = k_1./((x_1 - j).^2 + (y_1 - i).^2) + k_2./((x_2 - j).^2 + (y_2 - i).^2) + k_3./((x_3 - j).^2 + (y_3 - i).^2);
    end
end
mea_true_grid(:, 1) = -1;
mea_true_grid(1, :) = -1;
mea_true_grid(end, :) = -1;
mea_true_grid(:, end) = -1;
mea_true_grid(50:60, 10:30) = -1;
mea_true_grid(50:80, 50:60) = -1;

%% Set up occupuncy grid
occ_evi_grid = mea_true_grid == -1;

%% Experiment
% Initialize variables
y_0 = 92; % Starting row
x_0 = 54; % Starting col
mea_evi_grid = zeros(n, m) - 1;
mea_evi_grid(y_0, x_0) = mea_true_grid(y_0, x_0);
cat_grid = make_cat_grid(mea_evi_grid);
cat = cat_grid(y_0, x_0);
cur_y = y_0;
cur_x = x_0;
% T = 110; % Total steps
y_history = [y_0];
x_history = [x_0];
while ~check_source(cur_y, cur_x, mea_evi_grid)
    [next_y, next_x] = gradient_step(mea_evi_grid, occ_evi_grid, cur_y, cur_x, cat_grid, cat);
    cur_y = next_y;
    cur_x = next_x;
    y_history = [y_history, cur_y];
    x_history = [x_history, cur_x];
    mea_evi_grid(cur_y, cur_x) = mea_true_grid(cur_y, cur_x);
    cat_grid = make_cat_grid(mea_evi_grid);
    
    animate_cat_grid = cat_grid;
    animate_cat_grid(mea_true_grid == -1) = 0;
    animate_cat_grid(39, 52) = 0.5;
    animate_cat_grid(70, 79) = 0.5;
    animate_cat_grid(15, 26) = 0.5;
    heatmap(animate_cat_grid);
    pause(.1);

    cat = cat_grid(y_0, x_0);
end